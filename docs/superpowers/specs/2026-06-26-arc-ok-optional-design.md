# Design : ARC_OK physique facultatif avec fallback sur tension

**Date :** 2026-06-26  
**Auteur :** Kimi Code (agent conversationnel)  
**Projet :** SmartTHC — Torch Height Controller pour CNC plasma

## 1. Résumé

Rendre le signal `ARC_OK` physique (`PLASMA_PIN`, broche 12) facultatif via un build flag PlatformIO. Quand il est désactivé, le firmware considère l'arc comme présent dès que la tension plasma filtrée dépasse un seuil fixe de **50 V**.

## 2. Contexte

Actuellement, `THCController::updatePlasmaState()` lit `PLASMA_PIN` et dérive `plasmaPinLow` de l'état de cette broche :

```cpp
plasmaPinLow = (digitalRead(PLASMA_PIN) == LOW);
```

Toute la chaîne de gating (stabilisation, `arcDetected`, THC active, anti-dive, affichage des icônes) repose sur `plasmaPinLow`. L'utilisateur souhaite pouvoir fonctionner sans ce signal matériel en se basant sur la tension elle-même.

Première question de l'utilisateur : *l'écran affiche-t-il la tension si ARC_OK n'est pas activé ?*  
**Réponse : oui.** `DisplayManager::drawScreen0()` affiche `thc->getFastVoltage()` sans condition sur `plasmaPinLow`. Seuls les icônes de statut et l'activation du THC dépendent de `plasmaPinLow`.

## 3. Décisions de design

| Choix | Valeur retenue | Justification |
|-------|----------------|---------------|
| Build flag | `-D ARC_OK_PIN=1` par défaut, `-D ARC_OK_PIN=0` pour désactiver | Nom explicite : quand la valeur est 0, la broche physique est ignorée. |
| Seuil de fallback | `ARC_OK_FALLBACK_VOLTAGE = 50.0 V` | Demande initiale de l'utilisateur. |
| Localisation du seuil | `src/Config.h` | Concentrer les constantes au même endroit que les autres seuils. |
| Approche d'implémentation | Switch compile-time (`#if ARC_OK_PIN`) | Minimal, aucun surcoût runtime, pas de nouvelle méthode. |
| Interaction avec le reste du code | Aucune | `plasmaPinLow` reste la seule source de vérité ; les autres fonctions n'ont pas besoin d'être modifiées. |

## 4. Détails d'implémentation

### 4.1 `src/Config.h`

Ajouter après la définition de `PLASMA_VOLTAGE` (section PINS) ou dans la section thresholds :

```cpp
#ifndef ARC_OK_PIN
#define ARC_OK_PIN 1  // 1 = utiliser la broche PLASMA_PIN, 0 = fallback sur tension
#endif

#ifndef ARC_OK_FALLBACK_VOLTAGE
#define ARC_OK_FALLBACK_VOLTAGE 50.0f  // Seuil (V) utilisé quand ARC_OK_PIN=0
#endif
```

### 4.2 `platformio.ini`

Dans `[common].build_flags`, ajouter :

```ini
-D ARC_OK_PIN=1  ; 0 = ignorer la broche ARC_OK physique, utiliser le seuil de tension
```

Pour désactiver le signal physique, l'utilisateur modifiera cette ligne (ou la surchargera dans un environnement dédié) :

```ini
-D ARC_OK_PIN=0
```

### 4.3 `src/THCController.cpp`

#### `begin()`

Conditionner la configuration de la broche :

```cpp
#if ARC_OK_PIN
    pinMode(PLASMA_PIN, INPUT);
#endif
```

#### `updatePlasmaState()`

Conditionner la lecture de la broche :

```cpp
#if ARC_OK_PIN
    plasmaPinLow = (digitalRead(PLASMA_PIN) == LOW);
#else
    plasmaPinLow = (fastVoltage > ARC_OK_FALLBACK_VOLTAGE);
#endif
```

Le reste de la fonction (commande des switches `SWITCH1`/`SWITCH2`, gestion de la stabilisation) reste inchangé car il utilise `plasmaPinLow`.

### 4.4 Autres fichiers

- `src/DisplayManager.cpp` : **aucune modification** nécessaire. L'affichage de la tension est déjà indépendant de `plasmaPinLow`.
- `src/SerialCommand.cpp` : **aucune modification** nécessaire. Les labels d'état (`PLASMA_OFF`, etc.) sont dérivés de `plasmaPinLow`, qui reste la bonne abstraction.
- `src/main.cpp` : **aucune modification** nécessaire.

## 5. Fichiers impactés

- `src/Config.h`
- `src/THCController.cpp`
- `platformio.ini`

## 6. Vérification / tests

1. **Build des deux environnements** :
   ```bash
   pio build -e uno_r4_minima
   pio build -e uno_r4_minima_imperial
   ```

2. **Build avec ARC_OK désactivé** :
   Créer ou modifier un environnement temporaire avec `-D ARC_OK_PIN=0` et vérifier que le build passe sans warning.

3. **Test bench (mode ARC_OK physique)** :
   - Câbler `PLASMA_PIN` à LOW, appliquer une tension > 10 V sur `A0`.
   - Confirmer `state=THC_ACTIVE` quand tous les autres gates sont clairs.

4. **Test bench (mode ARC_OK par tension)** :
   - Lancer un build avec `-D ARC_OK_PIN=0`.
   - Laisser `PLASMA_PIN` flottant ou déconnecté.
   - Appliquer une tension > 50 V sur `A0`.
   - Confirmer que `plasmaPinLow` passe à `true` et que la stabilisation / THC se déclenchent normalement.

## 7. Considérations de sécurité

- **Z polarity** : le mode fallback ne change pas la logique de commande du moteur Z. La polarité reste réglée par `Z_DIR_INVERT`. Toujours vérifier sur banc avant coupure réelle.
- **Bruit de tension** : le seuil de 50 V est supérieur au `ARC_THRESHOLD` de 10 V utilisé pour `arcDetected`. Cela évite que le bruit ou la tension résiduelle après extinction ne fasse croire à un arc OK.
- **Anti-dive** : en mode fallback, `plasmaPinLow` et `arcDetected` peuvent devenir vrais quasi simultanément. La logique de stabilisation (`STABILIZATION_DELAY`) et le re-seed du filtre lent restent actifs et protègent contre les fausses manipulations.
- **Switches de sortie** : `SWITCH1` et `SWITCH2` continuent de refléter `plasmaPinLow`. En mode fallback, ils s'activent donc dès que la tension dépasse 50 V.

## 8. Notes

- Cette modification est entièrement résolue au moment de la compilation. Aucune structure runtime supplémentaire n'est ajoutée.
- Si l'utilisateur souhaite plus tard rendre le seuil configurable à l'exécution, il faudra l'ajouter comme paramètre EEPROM. Ce n'est pas dans le scope de cette mise à jour.
