# Instructions de Compilation du Rapport LaTeX

## 📄 Fichiers créés

- **rapport_stage.tex** : Document LaTeX principal du rapport de stage

## 🔧 Prérequis

Pour compiler le document LaTeX, vous avez besoin d'une distribution LaTeX complète :

### Sur Ubuntu/Debian
```bash
sudo apt-get update
sudo apt-get install texlive-latex-base texlive-latex-extra texlive-lang-french texlive-fonts-recommended
```

### Sur macOS
```bash
brew install --cask mactex
```

### Sur Windows
Téléchargez et installez [MiKTeX](https://miktex.org/) ou [TeX Live](https://www.tug.org/texlive/)

## 📦 Packages LaTeX utilisés

Le document utilise les packages suivants (tous inclus dans une installation LaTeX complète) :

- `inputenc` : Encodage UTF-8
- `babel` : Support de la langue française
- `graphicx` : Insertion d'images
- `amsmath`, `amssymb` : Formules mathématiques
- `listings` : Coloration syntaxique pour code C++
- `xcolor` : Gestion des couleurs
- `hyperref` : Liens hypertextes et références croisées
- `geometry` : Marges personnalisées
- `fancyhdr` : En-têtes et pieds de page
- `titlesec` : Formatage des titres
- `enumitem` : Listes personnalisées
- `setspace` : Interligne personnalisé

## 🚀 Compilation

### Méthode 1 : Compilation simple (recommandée)

```bash
cd /chemin/vers/drone_project
pdflatex rapport_stage.tex
pdflatex rapport_stage.tex
```

**Note** : Il faut compiler deux fois pour que la table des matières et les références croisées soient correctement générées.

### Méthode 2 : Avec script automatique

Créez un script `compile.sh` :

```bash
#!/bin/bash
echo "Compilation du rapport de stage..."
pdflatex -interaction=nonstopmode rapport_stage.tex
pdflatex -interaction=nonstopmode rapport_stage.tex
echo "Compilation terminée ! Le fichier rapport_stage.pdf a été généré."
```

Puis exécutez :
```bash
chmod +x compile.sh
./compile.sh
```

### Méthode 3 : Avec latexmk (compilation automatique complète)

```bash
latexmk -pdf rapport_stage.tex
```

## 📊 Structure du document

Le rapport de stage est organisé comme suit :

1. **Page de garde** : Informations sur l'étudiant, l'entreprise et le stage
2. **Engagement de non-plagiat** : Déclaration formelle signée
3. **Fiche de synthèse** : Abstract (anglais) et Résumé (français)
4. **Table des matières** : Générée automatiquement
5. **Section 1 : Introduction** : Contexte, problématique et contributions
6. **Section 2 : Étape 1 -- Prise en main** : Installation et configuration de l'environnement
7. **Section 3 : Étape 2 -- Atterrissage sur cible fixe** : Système avec contrôle proportionnel
8. **Section 4 : Étape 3 -- Atterrissage sur cible mobile** : Système avec contrôleur PID adaptatif
9. **Section 5 : Étape 4 -- Limites du système** : Analyse des limitations
10. **Section 6 : Impact écologique** : Réflexion sur l'impact environnemental
11. **Section 7 : Conclusion** : Synthèse et perspectives

## 🎨 Personnalisation

### Modifier les marges
Dans le préambule, ligne :
```latex
\usepackage[a4paper, margin=2.5cm]{geometry}
```

### Modifier l'interligne
Dans le préambule, ligne :
```latex
\setstretch{1.15}  % Changer la valeur (1.0 = simple, 1.5 = 1.5 interligne, 2.0 = double)
```

### Modifier la taille de police
Dans la première ligne :
```latex
\documentclass[12pt,a4paper]{article}  % Changer 12pt en 10pt, 11pt ou 12pt
```

### Ajouter des images
Pour insérer une image dans le document :

```latex
\begin{figure}[h]
    \centering
    \includegraphics[width=0.8\textwidth]{chemin/vers/image.png}
    \caption{Légende de l'image}
    \label{fig:mon_label}
\end{figure}
```

Puis référencer l'image dans le texte avec `\ref{fig:mon_label}`.

## 🐛 Résolution des problèmes

### Erreur : Package not found
Si un package est manquant, installez-le :
```bash
# Sur Ubuntu
sudo apt-get install texlive-latex-extra

# Avec tlmgr (TeX Live Manager)
tlmgr install <nom-du-package>
```

### Erreur : Undefined control sequence
Vérifiez que tous les packages sont correctement chargés dans le préambule.

### Table des matières vide
Assurez-vous de compiler deux fois le document.

### Problème d'encodage (caractères accentués)
Vérifiez que votre éditeur sauvegarde le fichier en UTF-8.

## 📝 Fichiers générés

Après compilation, plusieurs fichiers sont créés :

- `rapport_stage.pdf` : **Le document final (c'est celui que vous voulez !)**
- `rapport_stage.aux` : Fichier auxiliaire pour les références
- `rapport_stage.log` : Journal de compilation (utile pour déboguer)
- `rapport_stage.toc` : Table des matières
- `rapport_stage.out` : Fichier pour les hyperliens

Vous pouvez supprimer les fichiers auxiliaires avec :
```bash
rm rapport_stage.aux rapport_stage.log rapport_stage.toc rapport_stage.out
```

## 📖 Ressources

- [Documentation LaTeX (français)](https://fr.wikibooks.org/wiki/LaTeX)
- [Overleaf (éditeur LaTeX en ligne)](https://www.overleaf.com/)
- [CTAN (Comprehensive TeX Archive Network)](https://www.ctan.org/)

## ✅ Validation

Pour vérifier que le document compile correctement :

```bash
cd /chemin/vers/drone_project
pdflatex rapport_stage.tex
```

Si la compilation réussit, vous devriez voir :
```
Output written on rapport_stage.pdf (XX pages, YYYY bytes).
```

Le document PDF est maintenant prêt à être consulté !

## 📧 Support

Pour toute question sur le document LaTeX ou des problèmes de compilation, consultez la documentation LaTeX ou les forums spécialisés.
