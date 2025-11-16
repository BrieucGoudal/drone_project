/**
 * @file my_drone_pid.cpp
 * @brief Atterrissage ArUco avec contrôleur PID pour cible mobile
 *
 * Structure avec 4 parties :
 * 1. Gestion caméra Gazebo
 * 2. Détection ArUco
 * 3. Contrôleur PID
 * 4. Contrôle de vol
 */

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/gimbal/gimbal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>

#ifdef HAVE_GZ
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#endif

using namespace mavsdk;

// === PARTIE 1: CLASSE POUR CAMERA GAZEBO ===
#ifdef HAVE_GZ
class GazeboCamera {
public:
    GazeboCamera(const std::string& topic) {
        if (!node.Subscribe(topic, &GazeboCamera::on_image, this)) {
            throw std::runtime_error("Erreur connexion caméra Gazebo");
        }
        std::cout << "Caméra Gazebo connectée" << std::endl;
    }

    bool get_frame(cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_frame_.empty()) return false;
        frame = latest_frame_.clone();
        return true;
    }

private:
    void on_image(const gz::msgs::Image& msg) {
        if (msg.pixel_format_type() != gz::msgs::PixelFormatType::RGB_INT8) return;

        cv::Mat img(msg.height(), msg.width(), CV_8UC3);
        memcpy(img.data, msg.data().c_str(), msg.data().size());
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

        std::lock_guard<std::mutex> lock(mutex_);
        latest_frame_ = img;
    }

    gz::transport::Node node;
    cv::Mat latest_frame_;
    std::mutex mutex_;
};
#endif

// === PARTIE 2: CLASSE POUR DETECTION ARUCO ===
class ArucoDetector {
public:
    ArucoDetector() {
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        params_ = cv::aruco::DetectorParameters::create();

        // Paramètres optimisés pour détection à distance
        params_->minMarkerPerimeterRate = 0.01;
        params_->maxMarkerPerimeterRate = 4.0;
        params_->adaptiveThreshWinSizeMin = 3;
        params_->adaptiveThreshWinSizeMax = 23;
        params_->adaptiveThreshConstant = 7;
        params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        params_->cornerRefinementWinSize = 5;

        std::cout << "Détecteur ArUco initialisé" << std::endl;
    }

    bool detect(const cv::Mat& frame, cv::Point2f& center, int& marker_id) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;

        cv::aruco::detectMarkers(frame, dictionary_, corners, ids, params_, rejected);

        if (ids.empty()) return false;

        marker_id = ids[0];

        // Calculer le centre
        center = cv::Point2f(0, 0);
        for (const auto& corner : corners[0]) {
            center += corner;
        }
        center /= 4.0f;

        // Stocker pour affichage
        last_corners_ = corners;
        last_ids_ = ids;

        return true;
    }

    void draw_markers(cv::Mat& frame, const cv::Point2f& center, int marker_id) {
        if (!last_ids_.empty()) {
            cv::aruco::drawDetectedMarkers(frame, last_corners_, last_ids_);
        }

        cv::Point2f img_center(frame.cols/2.0f, frame.rows/2.0f);
        cv::circle(frame, img_center, 5, cv::Scalar(0,255,0), -1);
        cv::circle(frame, center, 5, cv::Scalar(0,0,255), -1);
        cv::line(frame, img_center, center, cv::Scalar(255,0,0), 2);
    }

private:
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> params_;
    std::vector<std::vector<cv::Point2f>> last_corners_;
    std::vector<int> last_ids_;
};

// === PARTIE 3: CONTROLEUR PID ===
class PIDController {
public:
    PIDController(float kp, float ki, float kd, float max_integral = 2.0f) 
        : kp_(kp), ki_(ki), kd_(kd), max_integral_(max_integral), 
          error_sum_(0), last_error_(0) {}

    float compute(float error, float dt) {
        // Terme P (Proportionnel) - Réactivité immédiate
        float p_term = kp_ * error;

        // Terme I (Intégral) - Correction des erreurs persistantes
        error_sum_ += error * dt;
        error_sum_ = std::clamp(error_sum_, -max_integral_, max_integral_);
        float i_term = ki_ * error_sum_;

        // Terme D (Dérivé) - Anticipation et stabilité
        float d_term = 0.0f;
        if (dt > 0) {
            d_term = kd_ * (error - last_error_) / dt;
        }
        last_error_ = error;

        return p_term + i_term + d_term;
    }

    void reset() {
        error_sum_ = 0;
        last_error_ = 0;
    }

    void set_gains(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

private:
    float kp_, ki_, kd_;
    float max_integral_;
    float error_sum_;
    float last_error_;
};

// === PARTIE 4: CLASSE POUR CONTROLE DRONE AVEC PID ===
class DroneController {
public:
    DroneController(std::shared_ptr<System> system) : system_(system) {
        action_ = std::make_unique<Action>(system);
        telemetry_ = std::make_unique<Telemetry>(system);
        offboard_ = std::make_unique<Offboard>(system);
        gimbal_ = std::make_unique<Gimbal>(system);

        // Initialisation des contrôleurs PID pour X et Y
        // Gains DOUX pour contrôle progressif : P=2.0, I=0.5, D=0.3
        pid_x_ = std::make_unique<PIDController>(2.0f, 0.5f, 0.3f);
        pid_y_ = std::make_unique<PIDController>(2.0f, 0.5f, 0.3f);

        last_time_ = std::chrono::steady_clock::now();
    }

    bool takeoff() {
        std::cout << "=== DÉCOLLAGE ===" << std::endl;

        while (!telemetry_->health_all_ok()) {
            std::cout << "Attente drone prêt..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        if (action_->arm() != Action::Result::Success) {
            std::cerr << "Erreur armement" << std::endl;
            return false;
        }

        if (action_->takeoff() != Action::Result::Success) {
            std::cerr << "Erreur décollage" << std::endl;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::seconds(8));

        // Monter à 10m
        std::cout << "Montée à altitude de travail..." << std::endl;
        Action::Result goto_result = action_->goto_location(
            telemetry_->position().latitude_deg,
            telemetry_->position().longitude_deg,
            10.0f,
            0.0f
        );

        if (goto_result == Action::Result::Success) {
            std::cout << "Montée vers 10m initiée..." << std::endl;
            while (telemetry_->position().relative_altitude_m < 9.0f) {
                std::cout << "Altitude actuelle: " << telemetry_->position().relative_altitude_m << "m" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            std::cout << "Altitude cible atteinte !" << std::endl;
        } else {
            std::cerr << "Erreur montée manuelle" << std::endl;
        }

        while (telemetry_->flight_mode() != Telemetry::FlightMode::Hold) {
            std::cout << "Attente stabilisation vol..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        std::cout << "Drone stabilisé à position initiale" << std::endl;

        // Configurer gimbal
        gimbal_->take_control(0, Gimbal::ControlMode::Primary);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        gimbal_->set_angles(0, 0.0f, -90.0f, 0.0f, Gimbal::GimbalMode::YawLock, Gimbal::SendMode::Once);

        std::cout << "Décollage terminé, caméra orientée vers le bas" << std::endl;
        return true;
    }

    bool start_offboard() {
        std::cout << "=== ACTIVATION OFFBOARD ===" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(2));

        Offboard::VelocityNedYaw cmd{0, 0, 0, 0};
        for (int i = 0; i < 20; i++) {
            offboard_->set_velocity_ned(cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        for (int attempt = 0; attempt < 3; attempt++) {
            auto result = offboard_->start();
            if (result == Offboard::Result::Success) {
                std::cout << "Mode offboard activé" << std::endl;
                return true;
            }

            std::cout << "Tentative " << (attempt + 1) << " échouée, retry..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));

            for (int i = 0; i < 10; i++) {
                offboard_->set_velocity_ned(cmd);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        std::cerr << "Erreur activation offboard après 3 tentatives" << std::endl;
        return false;
    }

    void control_flight(const cv::Point2f& aruco_center, const cv::Size& frame_size, bool aruco_detected) {
        float v_north = 0, v_east = 0, v_down = 0;
        float altitude = telemetry_->position().relative_altitude_m;

        // Calcul du temps écoulé pour PID
        auto current_time = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(current_time - last_time_).count();
        last_time_ = current_time;

        if (aruco_detected && altitude > 0.1f) {
            // === CONTRÔLE PID POUR CIBLE MOBILE ===
            cv::Point2f img_center(frame_size.width/2.0f, frame_size.height/2.0f);
            cv::Point2f error = aruco_center - img_center;

            // Normaliser erreur entre -1 et 1
            float norm_error_x = error.x / (frame_size.width/2.0f);
            float norm_error_y = error.y / (frame_size.height/2.0f);

            // Ajuster gains PID selon altitude - GAINS DOUX
            if (altitude > 2.0f) {
                // Haute altitude : gains modérés
                pid_x_->set_gains(2.0f, 0.5f, 0.3f);
                pid_y_->set_gains(2.0f, 0.5f, 0.3f);
            } else {
                // Basse altitude : gains très doux pour précision
                pid_x_->set_gains(1.5f, 0.3f, 0.2f);
                pid_y_->set_gains(1.5f, 0.3f, 0.2f);
            }

            // Calcul PID avec limitation de vitesse RÉDUITE
            float max_speed = (altitude > 2.0f) ? 2.0f : 1.5f;
            v_east = std::clamp(pid_x_->compute(norm_error_x, dt), -max_speed, max_speed);
            v_north = std::clamp(-pid_y_->compute(norm_error_y, dt), -max_speed, max_speed);

            // Descente DOUCE et progressive
            v_down = (altitude > 3.0f) ? 0.4f :
                     (altitude > 1.5f) ? 0.3f : 0.2f;

            std::cout << "CONTRÔLE PID - Alt: " << altitude << "m, Err: "
                      << norm_error_x << "," << norm_error_y
                      << ", Vitesses: " << v_east << "," << v_north << "," << v_down 
                      << ", dt: " << dt << "s" << std::endl;

        } else if (altitude > 0.8f) {
            // === PAS D'ARUCO - DESCENTE LENTE EN RECHERCHE ===
            // Continuer à descendre lentement en cherchant l'ArUco
            // jusqu'à 0.8m avant de passer en descente directe
            if (search_counter_ == 0) {
                std::cout << "Cible perdue - Descente lente en recherche" << std::endl;
                pid_x_->reset();
                pid_y_->reset();
            }

            v_down = 0.15f;  // Descente lente pour chercher ArUco

            if (++search_counter_ % 50 == 0) {
                std::cout << "Recherche ArUco en descente à " << altitude << "m" << std::endl;
            }

        } else {
            // === ALTITUDE < 0.8m - DESCENTE EN FLECHE ===
            std::cout << "Descente en flèche - Alt: " << altitude << "m" << std::endl;
            pid_x_->reset();
            pid_y_->reset();
            v_north = 0;
            v_east = 0;
            v_down = 0.5f;
        }

        // Reset compteur de recherche si ArUco détecté
        if (aruco_detected) {
            search_counter_ = 0;
        }

        // Envoyer commandes
        Offboard::VelocityNedYaw cmd{v_north, v_east, v_down, 0};
        offboard_->set_velocity_ned(cmd);

        // Atterrissage automatique
        if (altitude < 0.15f) {
            std::cout << "ATTERRISSAGE AUTOMATIQUE" << std::endl;
            action_->land();
            landing_triggered_ = true;
        }
    }

    float get_altitude() {
        return telemetry_->position().relative_altitude_m;
    }

    bool should_land() {
        return landing_triggered_;
    }

private:
    std::shared_ptr<System> system_;
    std::unique_ptr<Action> action_;
    std::unique_ptr<Telemetry> telemetry_;
    std::unique_ptr<Offboard> offboard_;
    std::unique_ptr<Gimbal> gimbal_;

    // Contrôleurs PID
    std::unique_ptr<PIDController> pid_x_;
    std::unique_ptr<PIDController> pid_y_;
    std::chrono::steady_clock::time_point last_time_;

    int search_counter_ = 0;
    bool landing_triggered_ = false;
    
    // Mémoire de trajectoire pour descente sous 3m
    float memorized_v_north_ = 0.0f;
    float memorized_v_east_ = 0.0f;
    bool trajectory_memorized_ = false;
};

// === PROGRAMME PRINCIPAL ===
int main() {
    std::cout << "=== DRONE ARUCO AVEC PID ===" << std::endl;

    // === CONNEXION DRONE ===
    Mavsdk mavsdk(Mavsdk::Configuration(ComponentType::GroundStation));
    mavsdk.add_any_connection("udpin://0.0.0.0:14540");

    std::shared_ptr<System> system;
    while (!system) {
        auto systems = mavsdk.systems();
        if (!systems.empty()) system = systems[0];
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Drone connecté!" << std::endl;

    // === INITIALISATION COMPOSANTS ===
#ifdef HAVE_GZ
    std::unique_ptr<GazeboCamera> camera;
    try {
        camera = std::make_unique<GazeboCamera>("/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image");
    } catch (const std::exception& e) {
        std::cerr << "Erreur caméra: " << e.what() << std::endl;
        return -1;
    }
#else
    std::cerr << "Support Gazebo requis!" << std::endl;
    return -1;
#endif

    ArucoDetector aruco_detector;
    DroneController drone_controller(system);

    // === DÉCOLLAGE ===
    if (!drone_controller.takeoff()) {
        return -1;
    }

    if (!drone_controller.start_offboard()) {
        return -1;
    }

    // === BOUCLE PRINCIPALE ===
    std::cout << "=== DÉBUT ATTERRISSAGE ARUCO AVEC PID ===" << std::endl;

    while (true) {
        // Obtenir image
        cv::Mat frame;
        if (!camera->get_frame(frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        // Détecter ArUco
        cv::Point2f aruco_center;
        int marker_id;
        bool aruco_detected = aruco_detector.detect(frame, aruco_center, marker_id);

        // Contrôler le drone avec PID
        drone_controller.control_flight(aruco_center, frame.size(), aruco_detected);

        // Affichage
        if (aruco_detected) {
            aruco_detector.draw_markers(frame, aruco_center, marker_id);
            std::string info = "ArUco ID:" + std::to_string(marker_id) +
                              " Alt:" + std::to_string(drone_controller.get_altitude()).substr(0,4) + "m (PID)";
            cv::putText(frame, info, cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255), 2);
        } else {
            cv::putText(frame, "Recherche ArUco... (PID)", cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,255), 2);
        }

        cv::imshow("Drone Camera - PID", frame);

        // Sortie
        if (cv::waitKey(1) == 27 || drone_controller.should_land()) {
            break;
        }
    }

    // === NETTOYAGE ===
    cv::destroyAllWindows();
    std::cout << "Programme terminé" << std::endl;
    return 0;
}
