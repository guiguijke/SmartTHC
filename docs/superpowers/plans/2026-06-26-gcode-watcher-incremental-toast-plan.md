> **I'm using the writing-plans skill to create the implementation plan.**

# GCODE Watcher — Traitement incrémental et notifications toast

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Modifier `GCODE_Watcher.exe` pour ne convertir que les fichiers créés après son lancement, afficher une notification toast au démarrage, et conserver la notification après conversion.

**Architecture:** Suppression de l’appel `process_existing()` au démarrage de `gcode_watcher.py` et ajout d’une notification via `pystray.Icon.notify()`. Mise à jour de `README.txt`. Recompilation avec `build_exe.bat`.

**Tech Stack:** Python 3.10+, pystray, watchdog, pyinstaller

---

## File structure

| File | Responsibility |
|------|----------------|
| `Premium_Documentation/GCODE TRANSFORMER/gcode_watcher.py` | Logique du daemon : surveillance, transformation, notifications. |
| `Premium_Documentation/GCODE TRANSFORMER/README.txt` | Documentation utilisateur. |
| `Premium_Documentation/GCODE TRANSFORMER/build_exe.bat` | Script de build PyInstaller. |
| `Premium_Documentation/GCODE TRANSFORMER/dist/GCODE_Watcher.exe` | Exécutable généré. |

---

## Task 1: Modifier `gcode_watcher.py`

**Files:**
- Modify: `Premium_Documentation/GCODE TRANSFORMER/gcode_watcher.py`

- [ ] **Step 1: Supprimer la méthode `process_existing()`**

  Repérer et supprimer toute la méthode `process_existing()` de la classe `GcodeHandler` :

  ```python
  def process_existing(self, source_dir):
      """Transform any files already present in the source folder."""
      if not os.path.isdir(source_dir):
          return
      for name in sorted(os.listdir(source_dir)):
          path = os.path.join(source_dir, name)
          if os.path.isfile(path) and is_gcode_file(path):
              logger.info(f"Processing existing file: {name}")
              self._process(path)
  ```

- [ ] **Step 2: Supprimer l’appel `process_existing()` dans `main()`**

  Repérer dans `main()` :

  ```python
  # Process files already in folder
  handler.process_existing(source_dir)
  ```

  Supprimer ces deux lignes.

- [ ] **Step 3: Ajouter la notification de démarrage**

  Juste avant `icon.run()`, ajouter :

  ```python
  handler._notify("GCODE Watcher démarré", f"Surveillance de {source_dir}")
  ```

  La section concernée doit ressembler à :

  ```python
  icon = pystray.Icon("GCODE Watcher", icon_image, "SmartTHC GCODE Watcher", menu)
  handler.set_icon(icon)

  handler._notify("GCODE Watcher démarré", f"Surveillance de {source_dir}")

  icon.run()
  ```

- [ ] **Step 4: Vérifier le fichier modifié**

  Run: `python -m py_compile "Premium_Documentation/GCODE TRANSFORMER/gcode_watcher.py"`
  Expected: aucune erreur, commande se termine avec code 0.

---

## Task 2: Mettre à jour `README.txt`

**Files:**
- Modify: `Premium_Documentation/GCODE TRANSFORMER/README.txt`

- [ ] **Step 1: Modifier la description de `GCODE_Watcher.exe`**

  Remplacer :

  ```text
  2. GCODE_Watcher.exe  (Background daemon - automatic)
     - Double-click to start (runs in system tray)
     - Creates two folders on your Desktop:
         Desktop\GCODE\Raw\          <-- drop files here
         Desktop\GCODE\Transformed\  <-- transformed files appear here
     - Every time you export from SheetCam to the Raw folder,
       the watcher automatically transforms the file once it is fully written
     - Re-exporting a file with the same name also re-triggers transformation
  ```

  Par :

  ```text
  2. GCODE_Watcher.exe  (Background daemon - automatic)
     - Double-click to start (runs in system tray)
     - Creates two folders on your Desktop:
         Desktop\GCODE\Raw\          <-- drop files here
         Desktop\GCODE\Transformed\  <-- transformed files appear here
     - Only files dropped into the Raw folder AFTER the watcher is started
       are transformed. Files already present at startup are ignored.
     - Every time you export from SheetCam to the Raw folder,
       the watcher automatically transforms the file once it is fully written
     - Re-exporting a file with the same name also re-triggers transformation
     - A toast notification is shown on startup and after each conversion
  ```

---

## Task 3: Recompiler l’exécutable

**Files:**
- Modify: `Premium_Documentation/GCODE TRANSFORMER/dist/GCODE_Watcher.exe` (généré)

- [ ] **Step 1: Lancer le build**

  Run:
  ```batch
  cd "Premium_Documentation/GCODE TRANSFORMER"
  build_exe.bat
  ```

  Expected: les trois builds PyInstaller se terminent avec succès, le fichier `dist/GCODE_Watcher.exe` est mis à jour.

- [ ] **Step 2: Vérifier la présence de l’exe**

  Run: `ls -la "Premium_Documentation/GCODE TRANSFORMER/dist/GCODE_Watcher.exe"`
  Expected: le fichier existe et sa date de modification est récente.

---

## Task 4: Test manuel rapide

- [ ] **Step 1: Préparer le dossier Raw**

  Créer `~/Desktop/GCODE/Raw` et y placer un vieux fichier `.nc` **avant** de lancer le watcher.

- [ ] **Step 2: Lancer `GCODE_Watcher.exe`**

  Double-cliquer sur `dist/GCODE_Watcher.exe`.

  Expected: une notification toast « GCODE Watcher démarré » apparaît. Le vieux fichier dans `Raw` n’est pas transformé.

- [ ] **Step 3: Déposer un nouveau fichier**

  Copier un fichier `.nc` dans `~/Desktop/GCODE/Raw` **après** le lancement.

  Expected: une notification toast « GCODE transformed! » apparaît et le fichier transformé apparaît dans `~/Desktop/GCODE/Transformed`.

---

## Self-review

- **Spec coverage:**
  - Suppression conversion massive au démarrage → Task 1, Step 1 et 2.
  - Notification toast au démarrage → Task 1, Step 3.
  - Notification après conversion → déjà présent, non modifié.
  - Mise à jour README → Task 2.
  - Rebuild de l’exe → Task 3.
  - Test manuel → Task 4.
- **Placeholder scan:** aucun TBD/TODO.
- **Type consistency:** pas de nouvelles signatures, `_notify()` existe déjà.
