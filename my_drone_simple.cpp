/**
 * @file my_drone_simple.cpp
 * @brief Atterrissage ArUco sur cible FIXE avec contrôle proportionnel simple (système P)
 * 
 * Version simplifiée sans PID complet, pour la Phase 2 du rapport.
 * Utilise uniquement des corrections proportionnelles à l'erreur.
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

        // Paramètres optimisés
        params_->minMarkerPerimeterRate = 0.01;
        params_->maxMarkerPerimeterRate = 4.0;
        params_->adaptiveThreshWinSizeMin = 3;
        params_->adaptiveThreshWinSizeMax = 23;
        params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        std::cout << "Détecteur ArUco initialisé" << std::endl;
    }

    bool detect(const cv::Mat& frame, cv::Point2f& center, int& marker_id) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(frame, dictionary_, corners, ids, params_);

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

// === PARTIE 3: CONTROLEUR SIMPLE (PROPORTIONNEL) ===
class SimpleController {
public:
    SimpleController(float kp, float centering_threshold) 
        : kp_(kp), centering_threshold_(centering_threshold) {}

    // Calcul correction proportionnelle simple
    void compute_velocities(float error_x, float error_y, float& v_east, float& v_north) {
        // Correction proportionnelle pure : vitesse = Kp * erreur
        v_east = kp_ * error_x;
        v_north = -kp_ * error_y;  // Inversé car axe Y image inversé

        // Limiter les vitesses maximales
        v_east = std::clamp(v_east, -0.5f, 0.5f);
        v_north = std::clamp(v_north, -0.5f, 0.5f);
    }

    // Vérifier si le drone est bien centré sur la cible
    bool is_centered(float error_x, float error_y) {
        float distance = std::sqrt(error_x * error_x + error_y * error_y);
        return distance < centering_threshold_;
    }

private:
    float kp_;                    // Gain proportionnel
    float centering_threshold_;   // Seuil de centrage en pixels
};

// === PARTIE 4: CONTROLE DE VOL ===
class DroneController {
public:
    DroneController(System& system) {
        action_ = std::make_unique<Action>(system);
        telemetry_ = std::make_unique<Telemetry>(system);
        offboard_ = std::make_unique<Offboard>(system);
        gimbal_ = std::make_unique<Gimbal>(system);
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
            while (telemetry_->position().relative_altitude_m < 9.0f) {
                std::cout << "Altitude: " << telemetry_->position().relative_altitude_m << "m" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        while (telemetry_->flight_mode() != Telemetry::FlightMode::Hold) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "Drone stabilisé" << std::endl;

        // Orienter caméra vers le bas
        gimbal_->take_control(0, Gimbal::ControlMode::Primary);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        gimbal_->set_angles(0, 0.0f, -90.0f, 0.0f, Gimbal::GimbalMode::YawLock, Gimbal::SendMode::Once);

        std::cout << "Caméra orientée vers le bas" << std::endl;
        return true;
    }

    bool start_offboard() {
        Offboard::VelocityNedYaw stay{};
        offboard_->set_velocity_ned(stay);

        Offboard::Result offboard_result = offboard_->start();
        if (offboard_result != Offboard::Result::Success) {
            std::cerr << "Erreur mode offboard" << std::endl;
            return false;
        }

        std::cout << "Mode offboard activé" << std::endl;
        return true;
    }

    void send_velocity(float v_north, float v_east, float v_down) {
        Offboard::VelocityNedYaw cmd{v_north, v_east, v_down, 0};
        offboard_->set_velocity_ned(cmd);
    }

    float get_altitude() {
        return telemetry_->position().relative_altitude_m;
    }

    void land() {
        std::cout << "ATTERRISSAGE" << std::endl;
        action_->land();
    }

private:
    std::unique_ptr<Action> action_;
    std::unique_ptr<Telemetry> telemetry_;
    std::unique_ptr<Offboard> offboard_;
    std::unique_ptr<Gimbal> gimbal_;
};

// === MAIN ===
int main(int argc, char** argv) {
    std::cout << "=== ATTERRISSAGE SIMPLE SUR CIBLE FIXE ===" << std::endl;

    // Connexion MAVSDK
    Mavsdk mavsdk(Mavsdk::Configuration(ComponentType::GroundStation));
    ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14540");

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Erreur connexion" << std::endl;
        return 1;
    }

    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Aucun drone détecté" << std::endl;
        return 1;
    }

    // Initialisation
    DroneController drone(*system.value());
    ArucoDetector detector;
    SimpleController controller(0.002f, 50.0f);  // Kp=0.002, seuil=50px

#ifdef HAVE_GZ
    std::unique_ptr<GazeboCamera> camera;
    try {
        camera = std::make_unique<GazeboCamera>("/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image");
    } catch (const std::exception& e) {
        std::cerr << "Erreur caméra: " << e.what() << std::endl;
        return 1;
    }
#endif

    // Décollage
    if (!drone.takeoff()) {
        return 1;
    }

    // Démarrer mode offboard
    if (!drone.start_offboard()) {
        return 1;
    }

    std::cout << "=== DÉBUT ATTERRISSAGE ===" << std::endl;

    cv::Mat frame;
    int frames_centered = 0;
    const int FRAMES_REQUIRED = 10;  // Rester centré 10 frames avant descente

    auto loop_start = std::chrono::steady_clock::now();

    while (true) {
        auto iteration_start = std::chrono::steady_clock::now();

        // Récupérer image
        if (!camera->get_frame(frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        float v_north = 0.0f, v_east = 0.0f, v_down = 0.0f;
        float altitude = drone.get_altitude();

        // TEST ALTITUDE EN PREMIER : En dessous de 1m, on ignore tout et on descend
        if (altitude < 1.0f) {
            v_north = 0.0f;
            v_east = 0.0f;
            v_down = 0.3f;  // Descente rapide en flèche
            std::cout << "<<< ALTITUDE < 1m >>> DESCENTE RAPIDE - Alt: " << altitude << "m" << std::endl;
            
            cv::putText(frame, "DESCENTE RAPIDE < 1m", 
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,0,255), 3);
            cv::putText(frame, "Alt: " + std::to_string(altitude).substr(0,4) + "m", 
                       cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255), 2);
        }
        // Au dessus de 1m : utiliser ArUco normalement
        else {
            cv::Point2f marker_center;
            int marker_id;
            bool marker_detected = detector.detect(frame, marker_center, marker_id);

            if (marker_detected) {
                // Calculer erreur en pixels
                cv::Point2f img_center(frame.cols/2.0f, frame.rows/2.0f);
                float error_x = marker_center.x - img_center.x;
                float error_y = marker_center.y - img_center.y;

                // Correction proportionnelle
                controller.compute_velocities(error_x, error_y, v_east, v_north);

                // Vérifier centrage
                if (controller.is_centered(error_x, error_y)) {
                    frames_centered++;
                    std::cout << "Centré (" << frames_centered << "/" << FRAMES_REQUIRED << ")" << std::endl;
                } else {
                    frames_centered = 0;
                }

                // Descente si bien centré
                if (frames_centered >= FRAMES_REQUIRED) {
                    v_down = 0.3f;  // Descente normale
                    std::cout << "DESCENTE - Alt: " << altitude << "m" << std::endl;
                }

                // Affichage
                detector.draw_markers(frame, marker_center, marker_id);
            } else {
                std::cout << "Marqueur perdu - Maintien position" << std::endl;
                frames_centered = 0;
            }
        }

        cv::imshow("Atterrissage Simple", frame);
        cv::waitKey(1);

        // Envoyer commandes
        drone.send_velocity(v_north, v_east, v_down);

        // Atterrissage final
        if (altitude < 0.3f) {
            std::cout << "Altitude finale atteinte - ATTERRISSAGE" << std::endl;
            drone.land();
            break;
        }

        // Boucle à 10 Hz
        auto iteration_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(iteration_end - iteration_start);
        auto sleep_time = std::chrono::milliseconds(100) - elapsed;
        if (sleep_time.count() > 0) {
            std::this_thread::sleep_for(sleep_time);
        }
    }

    cv::destroyAllWindows();
    std::cout << "=== MISSION TERMINÉE ===" << std::endl;
    return 0;
}
