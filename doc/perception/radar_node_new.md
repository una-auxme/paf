# Detaillierte Beschreibung des Radar-Nodes

## 1. Allgemeine Funktionalität
Der **Radar-Node** verarbeitet Radardaten von zwei an der Vorderseite des Fahrzeugs montierten Sensoren. Er empfängt die Sensordaten, filtert unerwünschte Bodenreflexionen, führt Clustering mit dem DBSCAN-Algorithmus durch und veröffentlicht die gefilterten und gruppierten Daten. 

## 2. Sensor-Konfiguration
Die Sensoren sind wie folgt konfiguriert:

| Sensor  | x   | y    | z   | Horizontal FOV | Vertical FOV |
|---------|-----|------|-----|----------------|--------------|
| RADAR0  | 2.0 | -1.5 | 0.5 | 25             | 0.1          |
| RADAR1  | -2.0 | -1.5 | 0.5 | 25             | 0.1          |

**Besonderheiten:** RADAR1 ist um 180° gedreht, um nach hinten zu schauen.

## 3. Hauptkomponenten

### 3.1 Initialisierung und ROS-Parameter
Beim Start des Radar-Nodes werden mehrere Parameter über `get_param` abgerufen, die das Verhalten des Nodes steuern:

- `~dbscan_eps`: Maximaler Abstand zwischen zwei Punkten in einem Cluster (Standard: `0.3`)
- `~dbscan_samples`: Minimale Anzahl an Punkten pro Cluster (Standard: `3`)
- `~data_buffered`: Gibt an, ob Sensordaten zwischengespeichert werden sollen (Standard: `False`)
- `~data_buffer_time`: Zeitintervall für die Verarbeitung zwischengespeicherter Daten (Standard: `0.1` Sekunden)
- `~enable_clustering`: Aktiviert oder deaktiviert das Clustering (Standard: `False`)
- `~enable_debug_info`: Aktiviert die Ausgabe zusätzlicher Debug-Informationen (Standard: `False`)
- `~accelerometer_arrow_size`: Skalierungsfaktor für die IMU-Pfeilvisualisierung (Standard: `2.0`)
- `~accelerometer_factor`: Skalierungsfaktor für die Berechnung des Pitch-Winkels (Standard: `0.05`)

### 3.2 Datenverarbeitung
- **Empfang von Sensordaten**: 
  - Die Sensordaten werden von `/carla/hero/RADAR0` und `/carla/hero/RADAR1` als `PointCloud2` empfangen.
  - Die Daten können entweder in einem Puffer gespeichert oder sofort verarbeitet werden.
  
- **IMU-Datenverarbeitung**: 
  - Die Neigung des Fahrzeugs wird anhand der IMU-Daten berechnet, um Bodenspiegelungen herauszufiltern.
  - IMU-Daten werden im `imu_callback` verarbeitet, wobei die letzten fünf Messwerte der x- und z-Beschleunigung gespeichert und gemittelt werden.
  - Der Pitch-Winkel wird aus `atan2(accel_x_avg, accel_z_avg)` berechnet und mit einem Skalierungsfaktor modifiziert.

- **Zeitsteuerung**: 
  - Die Funktion `time_check` stellt sicher, dass die Verarbeitung nur in festgelegten Zeitintervallen erfolgt.
  - Das Zeitintervall wird mit `self.data_buffer_time` (Standard: `0.1s`) konfiguriert.
  
### 3.3 Datenfilterung
- Punkte unterhalb der berechneten Bodenlinie werden herausgefiltert.
- Punkte außerhalb des max. Erfassungsbereichs (100m) werden entfernt.
- Die Funktion `filter_points` verwendet eine Maskierung basierend auf dem berechneten Pitch-Winkel.

### 3.4 Clustering
- DBSCAN wird zur Gruppierung von Radar-Punkten verwendet.
- **Cluster-Parameter:**
  - `eps` (max. Distanz zwischen Punkten in einem Cluster) = 0.3
  - `min_samples` (min. Anzahl an Punkten pro Cluster) = 3
- Durchschnittliche Geschwindigkeit pro Cluster wird berechnet.
- Das Clustering kann über den Parameter `~enable_clustering` deaktiviert werden.

### 3.5 Ausgabe der Daten
- **Gefilterte Punkte:** In `filtered_out_points` gespeichert und zur Visualisierung veröffentlicht.
- **Geclusterte Punkte:** Werden als `PointCloud2` publiziert.
- **Bounding Boxes:** Für jedes Cluster wird eine Achs-aligned Bounding Box (AABB) berechnet und in RViz dargestellt.
- **Cluster-Informationen:** Metadaten zu Clustern werden als JSON bereitgestellt.

## 4. ROS-Themen
| Thema | Typ | Beschreibung |
|----------------------------|---------------------------|--------------------------------------------------|
| `/carla/hero/RADAR0` | `sensor_msgs/PointCloud2` | Eingangsdaten von Radar 0 |
| `/carla/hero/RADAR1` | `sensor_msgs/PointCloud2` | Eingangsdaten von Radar 1 |
| `/paf/hero/Radar/Visualization` | `sensor_msgs/PointCloud2` | Visualisierung der Cluster-Punkte |
| `/paf/hero/Radar/Marker` | `visualization_msgs/MarkerArray` | Bounding Boxes der Cluster |
| `/paf/hero/Radar/clustered_points` | `mapping.msg.ClusteredPointsArray` | Geclusterte Radar-Punkte mit Geschwindigkeitswerten |
| `/paf/hero/Radar/ClusterInfo` | `std_msgs/String` | JSON mit Cluster-Informationen |
| `/paf/hero/IMU` | `sensor_msgs/Imu` | Eingangsdaten des IMU-Sensors |

## 5. Fazit
Dieser Radar-Node ermöglicht eine robuste Verarbeitung von Radarsignalen zur Objekterkennung. Durch das Clustering mit DBSCAN und die Integration von IMU-Daten wird die Qualität der Sensordaten verbessert. Die generierten Bounding Boxes und Visualisierungen erleichtern die Analyse der Umgebung.

