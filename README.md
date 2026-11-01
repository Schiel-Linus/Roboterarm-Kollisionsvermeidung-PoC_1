# Prädiktive Kollisionsvermeidung für Roboterarme | Proof of Concept (MediaPipe & PyBullet)

Ein Proof of Concept zur prädiktiven Kollisionsvermeidung in der Robotik: Handgesten werden über eine handelsübliche Webcam in Echtzeit erfasst und mit einem digitalen Zwilling in einer Physiksimulation synchronisiert. Drohende Kollisionen werden durch Look-Ahead-Algorithmen im Voraus berechnet und die Bewegung stufenlos und hardware-schonend gestoppt.



## Demo

[![Demo Video](https://img.youtube.com/vi/YKKIQn7EL8U/maxresdefault.jpg)](https://www.youtube.com/watch?v=YKKIQn7EL8U)

*(Klick auf das Bild öffnet das ungelistete Demo-Video auf YouTube)*



## Eigene Algorithmen & Kernfunktionen

Dieses Projekt geht über die reine Schnittstellen-Programmierung hinaus und implementiert eigene Logik für eine flüssige und sichere Hardware-Steuerung:

- **Look-Ahead Kollisions-Filter:** Die Software simuliert zukünftige Posen des Roboters in PyBullet, um den exakten Annäherungswinkel zu Hindernissen zu ermitteln.
- **Dynamische Bremszonen:** Statt eines abrupten Stopps wird ein dynamischer Bremsweg (Gelbe Zone) berechnet. Dieser skaliert in Echtzeit basierend auf der aktuellen Motorgeschwindigkeit und der maximal zulässigen Hardware-Verzögerung.
- **Mathematische Bahnplanung (S-Rampe):** Homing- und Aufklapp-Sequenzen nutzen Polynome 5. Grades zur Generierung von S-Rampen. Dies garantiert Beschleunigungen und Verzögerungen ohne Ruck (Jerk), um die mechanischen Komponenten des echten Roboterarms zu schonen.
- **Latenzunabhängige Kinematik:** Die Geschwindigkeitsberechnung erfolgt iterativ über Delta-Time (`dt`). Selbst bei schwankenden Loop-Zeiten des Prozessors bleibt die reale Bewegung des Roboterarms konstant und flüssig.
- **Echtzeit-Teleoperation:** Ableitung von mehrdimensionalen Steuerbefehlen aus einem 21-Punkte Hand-Skelettmodell (MediaPipe) inkl. Totzonen-Logik und Gesten-Erkennung (z.B. Zwei-Hand-Abbruchgeste).



## Verwendete Technologien

- **Sprache:** Python 3.12
- **Computer Vision:** Google MediaPipe, OpenCV
- **Simulation:** PyBullet
- **Architektur:** Objektorientiertes Systemdesign mit zentraler Manager-Klasse zur Entkopplung von Sensorik, Physik-Engine und Hardware-Schnittstelle (Arduino/Serial).



## Installation & Setup

1. Repository klonen:
   ```bash
   git clone [https://github.com/Schiel-Linus/Roboterarm_Kollisionsvermeidung.git](https://github.com/Schiel-Linus/Roboterarm_Kollisionsvermeidung.git)
   cd Roboterarm_Kollisionsvermeidung
   ```
   
2. Virtuelle Umgebung erstellen (optional, aber empfohlen):
   ```bash
   python -m venv venv
   source venv/Scripts/activate  # Windows
   ```

3. Abhängigkeiten installieren:
   ```bash
   pip install -r requirements.txt
   ```



## Nutzung
Das Hauptprogramm wird über die main.py gestartet. Der Pfad zur URDF-Datei des Roboters ist relativ hinterlegt und funktioniert, solange die Ordnerstruktur beibehalten wird.
```bash
python main.py
```

*Hinweis: Das Terminal gibt während der Laufzeit ein Live-Dashboard mit aktuellen Abständen, dynamischen Bremswegen und dem prozentualen Systemschub aus.*

---

## Geplante Weiterentwicklung (Roadmap)

- [ ] **Inverse Kinematik (IK):** Umstellung von direkter Achsen-Teleoperation auf zielpunktbasierte Bahnplanung.
- [ ] **Dynamische Hindernisse:** Kollisionsprädiktion für bewegte Objekte im Arbeitsraum.
- [ ] **Hardware-Schnittstelle:** Übertragung der Steuerungsdaten auf reale Robotik-Hardware (z. B. via Arduino / Serial).








   

   
