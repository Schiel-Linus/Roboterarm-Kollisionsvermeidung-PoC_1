"""
=========================================================
PROJEKT:        Roboterarm mit prädiktiver Kollisionsvermeidung
DATEI:          Manager_Milestone_5.py
BESCHREIBUNG:   Zentrale Steuerungseinheit. Initialisiert und 
                koordiniert alle Subsysteme (Vision, Arduino, 
                Kinematik, Digital Twin, Kollisionsvermeidung) 
                und taktet die Endlosschleife des Systems.
---------------------------------------------------------
ENTWICKLER:     Linus Schiel
VERSION:        v1.0.0 (Proof of Concept 1)
=========================================================
"""

from Vision import Vision
from Arduino import Arduino
from Kinematik import Kinematik
from Digital_Twin import Digital_Twin
from Kollisionsvermeidung import Kollisionsvermeidung


class Manager:

    def __init__(self, urdf_pfad, kollisionsvermeidung_aktiv):
        # im Konstruktor wird zum jetzigen Zeitpunkt nur der URDF Pfad übergeben, später könnte noch der jeweilige COM Port für den Arduino und die Baudrate übergeben werden
        # zusätzlich wird ein Attribut übergeben was in der Main eingestellt ist --> ein Schalter welcher die Kollisionsvermeidung für die ersten Tests ausschalten kann
        print("System wird hochgefahren")

        # 0. Bypass --> Kollisionsvermeidung an/aus
        self.kollisionsvermeidung_aktiv = kollisionsvermeidung_aktiv

        # 1. Vision initialisieren (Kameraverbindung)
        self.kamera = Vision()

        # 2. Hardware Schnittstelle initialisieren (Arduino)
        self.arduino = Arduino()

        # 3. Kinematik initialisieren
        self.kinematik = Kinematik()

        # 4. Digitalen Zwilling in PyBullet initialisieren
        self.twin = Digital_Twin(urdf_pfad)

        # 5. Kollisionsvermeidung initialisieren
        # Nur initialisieren wenn Bypass und daher Kollisionsvermeidung für erste Tests nicht aktiv ist (Attribut self.kollisionsvermeidung=False)
        if self.kollisionsvermeidung_aktiv:
            # Achtung hier muss dem Konstruktor noch die max Beschleunigung/ Verzoegerung in Form einer Zahl übergeben werden im nächsten Milestone ist das so nicht mehr vorgesehen
            # --> Ziel ist eine .txt Datei in welcher solche Parameter (zusätzlich bspw. max_winkel_limit, min_winkel_limit, startwinkel, ...) hinterlegt und dann direkt an die jeweiligen KLassen übergeben werden
            self.sicherheits_filter = Kollisionsvermeidung(
                digital_twin=self.twin,             # Übergabe des digital twin an den Konstruktor sodass dieser spezifische Methode des twin aufrufen kann
                max_verzoegerung= 60.0,               # Max. erlaubte Bremsverzögerung, FAktor muss noch angepasst werden
                look_ahead_time=0.05,                # 0.5 Sekunden in die Zukunft simulieren
                sicherheitsabstand=0.02             # 2 cm rote Zone
            )

            print("Feature aktiviert: Prädiktive Kollisionsvermeidung")
        else:
            print("Feature deaktiviert: Bypass für Kollisionsvermeidung aktiv")

        # HInweis: Hier in der init könnten später Hindernisse geladen werden
        # NUR sinnvoll wenn Kollisionsvermeidung eingeschaltet wurde
        # self.twin.hindernisse_erzeugen([0.1, 0.1, 0.1], [0.3, 0.0, 0.1])
        self.twin.hindernisse_erzeugen([0.05,1.2,0.25],[-0.2,0.0,0.125])

        print("--- System ist bereit ---")


    def systemzyklus_ausfuehren(self):
        # Herz des Roboters
        # --> die main ruft diese Methode auf und übergibt damit die komplette Kontrolle an den Manager

        # Nutzung einer Endlosschleife --> wird von innen heraus gesprengt/beendet
        while True:

            # ==========================================
            # SCHRITT 1: SENSORIK (Kamera auslesen)
            # ==========================================
            aktuelles_bild, geschwindigkeits_steuerbefehle, abbruch_geste = self.kamera.errechne_Steuerbefehle()

            # Hardware-Fail-Safe: Wenn die Kamera absteckt, sofortiger Abbruch
            if aktuelles_bild is None:
                print("Hardware-Fehler: Kamera liefert kein Bild. Notabschaltung.")
                break 

            # User-Interface aktualisieren (Totzonen zeichnen & Bild anzeigen)
            aktuelles_bild = self.kamera.zeichne_totzone(aktuelles_bild)
            abbruch_taste = self.kamera.zeige_bild_und_pruefe_abbruch(aktuelles_bild)

            # Abbruchbedingungen bündeln
            # zwei Möglichkeiten Programm zu beenden:
            # 1. Abbruchgeste
            # 2. Taste q
            # --> wenn einer der beiden Werte True ist wird abbruchsignal auf True gesetzt
            abbruch_signal = abbruch_geste or abbruch_taste


            # ==========================================
            # SCHRITT 2: SICHERHEIT (Der neue Filter-Schritt)
            # ==========================================
            # Sicherheitsfilter darf nur in der Teleoperation verwendet werden, wenn Aufklapp-Modus aktiv (S-Rampe) ist die Annahme das keine Hindernisse im Weg sind
            

            # um dies zu gewährleisten wird zuvor eine Abfrage gemacht ob die Teleoperation aktiv ist
            # zusätzlich müssen geschwindigkeitssteuerbefehle errechnet sein

            # es wird standardmäßig das Objekt geschwindigkeits_steuerbefehle als sichere_steuerbefehle verwendet sodass es beim späteren aufrufen zu keinen Konflikten kommt wenn sichere_steuerbefehle nicht erstellt wäre
            # weil der Modus aufklappen noch aktiv ist
            sichere_steuerbefehle = geschwindigkeits_steuerbefehle

            if self.kinematik.aktueller_modus == "TELEOPERATION" and geschwindigkeits_steuerbefehle is not None:
                # ist beides gewährleistet überprüft der Sicherheitsfilter die Geschwindigkeit und drosselt diese bei Bedarf

                # zusätzliche Prüfung ob Bypass aktiviert wurde
                if self.kollisionsvermeidung_aktiv:
                    sichere_steuerbefehle = self.sicherheits_filter.sicherheitsfilter_anwenden(
                        ziel_geschwindigkeiten=geschwindigkeits_steuerbefehle,
                        jetzige_geschwindigkeiten=self.kinematik.aktuelle_geschwindigkeiten,
                        jetzige_winkel=self.kinematik.aktuelle_winkel
                    )


            # ==========================================
            # SCHRITT 3: KINEMATIK (Winkel berechnen)
            # ==========================================
            # Kinematik bekommt jetzt NICHT mehr die rohen Kamera-Befehle, 
            # sondern die durch den Sicherheitsfilter bereinigten Befehle

            aktualisierte_winkel = self.kinematik.update_zyklus(sichere_steuerbefehle, abbruch_signal)



            # ==========================================
            # SCHRITT 4: AUSFÜHRUNG (Twin & Hardware updaten)
            # ==========================================
            # digitale Twin erhält Winkelsequenz und setzt/ fährt die Gelenke auf die jeweiligen Winkel
            # zum einen für die Hindernisserkennung wichtig, und andererseits für die Benutzeroberfläche
            self.twin.aktualisiere_twin(aktualisierte_winkel)

            
            # Winkelsequenz geht an den Arduino der die Winkel beim physischen Roboterarm anpasst
            self.arduino.Datenpakete_senden(aktualisierte_winkel)


            # ==========================================
            # SCHRITT 5: SYSTEM-STATUS PRÜFEN
            # ==========================================
            # erst wenn der Roboterarm komplett in der Einklappposition ist darf die while Schleife gesprengt und das System komplett heruntergefahren werden
            if self.kinematik.aktueller_modus == "BEENDET":
                print("Kinematik meldet: Einklappen beendet. Fahre System herunter.")
                break


        # wenn While Schleife gesprengt wird (durch Fehler oder reguläres Einklappen) räumt Manager auf
        self.beende_system()


    def beende_system(self):
        # # Hier wird sichergestellt, dass keine Hardware in einem undefinierten Zustand bleibt.
        print("--- Starte Aufräum-Sequenz ---")
        self.kamera.verbindung_kamera_trennen()
        self.arduino.Verbindung_Arduino_trennen()
        self.twin.trennen()
        print("--- System sicher beendet ---")

