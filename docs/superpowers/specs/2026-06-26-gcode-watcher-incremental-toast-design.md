# GCODE Watcher — Traitement incrémental et notifications toast

## Contexte

`GCODE_Watcher.exe` est le daemon system tray du projet SmartTHC qui surveille le dossier `Raw` et transforme automatiquement les fichiers GCODE pour FluidNC. Actuellement, au lancement, il convertit **tous** les fichiers déjà présents dans `Raw` via `process_existing()`. L'utilisateur souhaite qu'il ne traite que les fichiers déposés/modifiés **après** le lancement, et qu'il affiche une notification toast Windows au démarrage ainsi qu'après chaque conversion.

## Objectifs

1. Supprimer la conversion massive des fichiers existants au démarrage.
2. Afficher une notification toast "GCODE Watcher démarré" au lancement.
3. Conserver la notification toast "GCODE transformed!" après chaque conversion (déjà implémentée).
4. Mettre à jour la documentation utilisateur (`README.txt`).
5. Recompiler `GCODE_Watcher.exe` via `build_exe.bat`.

## Approche retenue

Approche minimaliste (A) : supprimer l'appel à `process_existing()` au démarrage. Aucune option de configuration n'est ajoutée. C'est le changement le plus simple et direct.

## Fichiers modifiés

- `Premium_Documentation/GCODE TRANSFORMER/gcode_watcher.py`
- `Premium_Documentation/GCODE TRANSFORMER/README.txt`

## Modifications détaillées

### `gcode_watcher.py`

1. Dans `main()`, supprimer l'appel :
   ```python
   handler.process_existing(source_dir)
   ```
2. Supprimer la méthode `process_existing()` de la classe `GcodeHandler` car elle n'est plus utilisée.
3. Juste avant `icon.run()`, ajouter :
   ```python
   handler._notify("GCODE Watcher démarré", f"Surveillance de {source_dir}")
   ```

### `README.txt`

- Mettre à jour la description de `GCODE_Watcher.exe` pour indiquer que seuls les fichiers déposés **après** le lancement du watcher sont transformés.

## Gestion des erreurs

- Si la notification de démarrage échoue (icône non initialisée), elle est silencieusement ignorée via le `try/except` déjà présent dans `_notify()`.
- Si un fichier ne devient pas stable dans le délai imparti, la notification "File ignored (timeout)" est affichée.

## Tests / vérification

1. Lancer `build_exe.bat` pour recompiler.
2. Vérifier que `dist/GCODE_Watcher.exe` est bien mis à jour.
3. Placer un fichier `.nc` dans `Raw` **avant** de lancer l'exe : il ne doit pas être converti.
4. Déposer un nouveau fichier `.nc` dans `Raw` **après** le lancement : il doit être transformé et une notification doit apparaître.
5. Vérifier qu'une notification "GCODE Watcher démarré" s'affiche au lancement.

## Notes

- Les notifications passent par `pystray.Icon.notify()`, qui utilise les toasts natifs Windows en bas à droite.
- Aucune dépendance supplémentaire n'est requise.
