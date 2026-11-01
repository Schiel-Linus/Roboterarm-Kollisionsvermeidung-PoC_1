"""
=========================================================
PROJEKT:        Roboterarm mit prädiktiver Kollisionsvermeidung
DATEI:          Arduino_Milestone_5.py
BESCHREIBUNG:   Hardware-Schnittstelle zur physischen Welt. 
                Baut die serielle Kommunikation auf und verpackt 
                die berechneten Winkel-Arrays in normierte 
                String-Pakete für die Servomotoren.
---------------------------------------------------------
ENTWICKLER:     Linus Schiel
VERSION:        v1.0.0 (Proof of Concept 1)
=========================================================
"""

import serial

class Arduino:

    def __init__(self):

        # Übermittlung der Daten an Arduino
        # um Verbindung aufzubauen Nutzung des Befehls serial.Serial(port,baudrate)
        # --> Befehl führt zu Fehlermeldung wenn kein Gerät angeschlossen ist --> Nutzung von try & except

        try:
            # Python geht nach und nach die Zeilen im try Block durch wenn alle Befehle funktionieren und keine Fehlermeldungen auftauchen wird mit dem Code einfach weitergemacht (ohne except Block)
            # wenn Fehler entsteht (im try Block) Sprung in den except Block
            self.verbindung_arduino = serial.Serial('COM3', 9600)

            # um Später nur zu senden wenn Verbindung online ist wird boolsche Variable genutzt
            self.verbindung_Arduino_online = True

            # Um später beim debuggen leichter auf die Fehler zu kommen wird string im Terminal ausgeben ob Verbindung erfolgreich ist
            print("Verbindung mit Arduino augebaut")


        except:         # wenn in zweiter Zeile Fehler wegen nicht aufkommender Verbindung entsteht wird in den except Block gesprungen sodass Code dennoch weiter ausgeführt werden kann

            # Platzhalter sodass Variable auch besteht wenn except Block durchgelaufen ist
            self.verbindung_arduino = None

            # sodass keine Daten an Arduino übermittelt werden Variable auf False
            self.verbindung_Arduino_online = False
            # Fehlermeldung im Terminal
            print("Verbindung konnte nicht hergestellt werden")


    def Datenpakete_senden(self, winkel_liste):
        # der string für die serielle Kommunikation wird zusammengebaut
        befehl_string = f"<{winkel_liste[0]},{winkel_liste[1]},{winkel_liste[2]},{winkel_liste[3]},{winkel_liste[4]},{winkel_liste[5]}>\n"
        

        # Nur wenn Verbindung zu Arduino online und finger_ausgestreckt kein None als Wert hält (Finger Anzahl hat sich dann nicht verändert) senden der Daten
        if self.verbindung_Arduino_online and befehl_string is not None:      # self.verbindung_Arduino_online == True --> True ist irrelevant
            

            # durch diesen Befehl wird String an arduino übermittelt, WICHTIG: encode sodass string in Einser und Nullen umgewandelt wird
            self.verbindung_arduino.write(befehl_string.encode())  


    def Verbindung_Arduino_trennen(self):
        # Serielle Verbindung schließen (sofern diese offen war)
        if self.verbindung_Arduino_online: # if self.verbindung_Arduino_online == True --> True ist irrelevant
            self.verbindung_arduino.close()
        
            print("Arduino Verbindung geschlossen")

    