"""
=========================================================
PROJEKT:        Roboterarm mit prädiktiver Kollisionsvermeidung
DATEI:          Vision_Milestone_5.py
BESCHREIBUNG:   Kamerabasierte Gestensteuerung (Teleoperation). 
                Nutzt OpenCV und MediaPipe zum Hand-Tracking, 
                wertet Totzonen aus, erkennt Abbruchgesten und 
                berechnet die rohen Geschwindigkeits-Steuerbefehle.
---------------------------------------------------------
ENTWICKLER:     Linus Schiel
VERSION:        v1.0.0 (Proof of Concept 1)
=========================================================
"""


import cv2 # Bibliothek für die Bildverarbeitung
import mediapipe as mp # Bibliothek für Bilderkennung
import time # Bibliothek um Zeit nutzen zu können



class Vision:


    def __init__(self):

        #Verbindung zur Kamera herstellen
        self.kamera = cv2.VideoCapture(0) # Index ist 0 --> verbaute Kamera des Laptops

        # definieren der Bild Breite --> wird nachher zusätzlich genutzt um bei Ux Experience die Totzone Kasten zu berechnen
        self.bild_breite = 1280
        self.bild_hoehe = 720



        self.kamera.set(cv2.CAP_PROP_FRAME_WIDTH, self.bild_breite)
        self.kamera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.bild_hoehe)

        # Nachfolgender Teil wäre eine Möglichkeit um ein noch größeres Sichtfeld der Kamera zu erreichen --> wird aber Geschwindigkeit von media Pipe drücken, viel mehr Pixel zum analysieren
        #kamera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        #kamera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        # Spezielles Modul für Hände wird geladen
        self.mp_haende = mp.solutions.hands

        # Spezielles Modul um auf dem Bild zu zeichnen
        self.mp_zeichnen = mp.solutions.drawing_utils

        # eigentliche KI wird geladen und wird in Variable haende_erkennung geladen
        self.haende_erkennung = self.mp_haende.Hands()

        # sodass eine bestimmte Bewegung (Geste) das Programm beendet wird Zustandsvariable benötigt da Geste in zwei Bwegungen unterteilt ist
        self.abbruchbedingung_erster_teil_erfüllt = 0 #wenn erster Teil erfüllt ist wird VAriable auf 1 gesetzt

        # da die Abbruchbewegung in einem bestimmten Zeitraum passieren muss wird eine "Stoppuhr" benötigt
        self.stoppuhr_abbruchbedingung = 0


        # Totzonen müssen definiert und die Grenzen bzw. Punkte für die Methoden errechne_Stuerbefehle und zeige Totzone müssen als Attribut gespeichert sein

        # festlegen von Parameter für das Aussehen der Totzone Kasten formulieren
        self.box_breite = 300
        self.box_hoehe = 400
        self.abstand_rand_horizontal = 100   # Abstand zum linken bzw. rechten Bildschirmrand
        self.abstand_rand_vertikal = 150     # Abstand zum oberen bzw. unteren Bildschirmrand

        # Achtung: Das Koordinatensystem ist gedreht --> Ursprung ist oben links
        # nachfolgende Attribute sind für die Geschwindigkeitsberechnung relevant
        self.bildschirmrand_links = 0
        self.bildschirmrand_oben = 0
        self.bildschirmrand_rechts = self.bild_breite
        self.bildschirmrand_unten = self.bild_hoehe
        self.bildschirm_mitte = self.bild_breite / 2

        # Herleitung für Koordinaten siehe pdf --> Totzonen
        # Koordinaten für linken Kasten berechnen
        self.e1_x = self.abstand_rand_horizontal
        self.e1_y = self.abstand_rand_vertikal
        self.e2_x = self.abstand_rand_horizontal + self.box_breite
        self.e2_y = self.abstand_rand_vertikal + self.box_hoehe

        # Koordinaten Rechter Kasten berechnen
        self.e3_x = self.bild_breite - self.abstand_rand_horizontal
        self.e3_y = self.abstand_rand_vertikal
        self.e4_x = self.bild_breite - self.abstand_rand_horizontal - self.box_breite
        self.e4_y = self.abstand_rand_vertikal + self.box_hoehe
        

        # die Maximal Geschwindigkeit welche der Servo fahren wird, wird hier definiert
        # Achtung: dieser Wert wird später in der Hilfsmethode berechne Geschwindigkeit genutzt
        # Anpassungen müssen dennoch mit dem Arduino Code und in Abhängigkeit der Motoren gemacht werden
        self.max_speed_servo_1 = 10
        self.max_speed_servo_2 = 10
        self.max_speed_servo_3 = 10
        self.max_speed_servo_4 = 10
        self.max_speed_servo_5 = 10
        self.max_speed_servo_6 = 10


        # da die rechte Hand beim Faust zu machen den Greifarm schließen soll aber die Faust nicht durchgehen gehalten werden kann wird ein "Gedächtnis" benötigt
        self.greifer_ist_zu = False
        
        # wichtig ist hierbei dass wenn der Greifarm zu ist erst wieder aufgeht wenn nochmal eine Faust gemacht wird
        # jedoch wird durch mindestens alle 30 mili sekunden ein Bild gemacht was dazuführen würde wenn die Faust geschlossen wäre und sie dann wieder geöffnet werden würde würde Media Pipe sich im Kreis drehen
        # --> daher Nutzung einer weiteren Variable welche speichert ob im Bild davor auch schon eine Faust war
        self.letztes_bild_war_faust = False



    def errechne_Steuerbefehle(self):
        # definieren von Zustandsvariablen für die Übermittlung an die Motoren
        # da lediglich mit zwei return in der Methode gearbeitet wird und nicht immer eine Hand im Bild oder eine Bewegung stattfindet muss das return dennoch einen Wert zurückgeben können
        # daher wird ein einfaches Null den jeweligen Motor Steuerbefehlen zugeordnet
        motor_1_steuerbefehl = 0
        motor_2_steuerbefehl = 0
        motor_3_steuerbefehl = 0
        motor_4_steuerbefehl = 0
        motor_5_steuerbefehl = 0

        # gilt nicht für den Servo Motor 6 bei diesem muss der Steuerbefehl in jedem Moment in Abhängigkeit von dem Gedächtnis "berechnet" werden
        # Beispiel: wenn die while Schleife in main wieder von vorne beginnt und errechne steuerbefehle neu beginnt werden erstmal alle Steuerbefehle auf null gesetzt, wenn der Greifarm aber davor geschlossen wäre und die Faust nicht gemacht werden würde,
        # würde der Steuerbefehl einfach bei 0 bleiben und der Greifarm würde ungewollt geöffnet werden
        if self.greifer_ist_zu == True:
            motor_6_steuerbefehl = -1 * self.max_speed_servo_6
        else:
            motor_6_steuerbefehl = self.max_speed_servo_6


        # um zu prüfen ob bei den jeweiligen Händen eine Faust gemacht wird werden Variablen gebraucht welche zu Beginn der Methode immer auf Null gesetzt sind
        finger_nicht_ausgestreckt_linke_hand = 0
        finger_nicht_ausgestreckt_rechte_hand = 0


        # Befehl um die Kamera auszulösen --> kamera.read()
        # Befehl kann durch boolsche Variable zurückmelden ob Bild erfolgreich gemacht wurde
        # Bild selbst (aktuelles Bild) --> "Riesige Excel Tabelle mit Farbwerten"
        erfolg, aktuelles_bild = self.kamera.read()

        # Wenn das Bild aufnehmen nicht erfolgreich war muss die Methode sofort abgebrochen werden um Fehler zu vermeiden
        # um zu vehindern das weitere Fehler auftreten
        if not erfolg: 
            print("Warnung: Kamerafehler. Breche Bildauswertung ab.")
            
            # Rückgabe von None
            # Hierbei werden drei None zurückgegeben da die Main datei drei Datenpakete erwartet
            # --> es konnte aber kein Bild gemacht werden daher gibt es kein aktuelles Bild, kein Steuerbefehl, Abbruchbedinung wurde nicht erfüllt = None
            return None, None, None
        

        # spiegeln des Kamerabildes sodass der Nutzer bei der Hand nicht umdenken muss --> wenn er die rechte Hand hebt geht auch auf dem Bildschirm von ihm aus gesehen die rechte Hand hoch
        # Bild horizontal spiegeln --> cv.flip(aktuelles_bild,1) --> 1 = horizontal, 0 = vertikal
        aktuelles_bild = cv2.flip(aktuelles_bild, 1)


        # Open CV liest Bilder im BGR Format (Blau-Grün-Rot), Media Pipe benötigt aber im RGB Format (Rot-Grün-Blau)
        # --> Farben werden geändert
        bild_rgb = cv2.cvtColor(aktuelles_bild, cv2.COLOR_BGR2RGB)

        # Media Pipe erhält passend gefärbtes Bild --> Bild wird analysiert und alles was gefunden wird wird in Variablen ergebnis gespeichert
        ergebnis = self.haende_erkennung.process(bild_rgb)

        # Text Output wenn Hand im Bild ist
        # da in Variable ergebnis viele Ordner gespeichert sind, wird hier jetzt der Ordner mit den x/y/z Koordinaten der Gelenke ausgegeben
        # print(ergebnis.multi_hand_landmarks)

        # Wert welcher zurückgegeben wird sodass main die while Schleife beenden könnte
        abbruch_geste_erkannt = False


        # Achtung: dieses Objekt sollte wahrscheinlich herausgenommen werden da es irrelevant ist da bei direktem Abbruch einfach nullen zurückgegeben werden
        # es könnte passieren das Abbruchgeste direkt nach dem Start des Programms gezogen wird --> keine Steuerbefehle
        # dass bei Rückgabe der Werte (Steuerbefehle) kein Fehler geschmissen wird, wird Objekt zu Beginn auf False gesetzt
        steuerbefehle = None


        # Nachfolgender Block wird nur ausgeführt wenn Media Pipe eine Hand erkennt
        if ergebnis.multi_hand_landmarks:
            
            #zuerst wird hier auf Abbruchgeste überprüft --> dafür müssen zwei Hände im Bild sein sonst ist Geste nicht möglich
            # Befehl len() überprüft länge der Liste --> mediapipe erstellt je Hand eine Liste 
            if len(ergebnis.multi_hand_landmarks) == 2:
                # um die jeweiligen Hände nicht immer über ergebnis.multi_hand_landmarks[0] aufrufen zu müssen --> Nutzung von zweier Variablen
                # zum jetzigen Zeitpunkt wird lediglich eine Landmark für die Abbruchbedingung überprüft daher werden den beiden VAriablen direkt die landmarks des Mittelhandknochen zugeordnet (landmark 9)
                # später könnte System verbessert werden indem mehr landmarks miteinbezogen werden dann dürfte Zuordnung der landmark 9 nicht direkt zur jeweiligen Hand erfolgen
                hand_eins = ergebnis.multi_hand_landmarks[0].landmark[9].x
                hand_zwei = ergebnis.multi_hand_landmarks[1].landmark[9].x

                # für die Geste wird lediglich die horizontale Achse (x Achse) genutzt
                # --> Koordinatensystem von links nach rechs:   linke Rand des Bilds = 0.0
                #                                               rechte Rand des Bilds = 1.0

                # Geste für Abbruchbedinung ist wie folgt definiert: Zusammenführen der Hände bis diese sich berühren und danach direkt wieder auseinanderführen
                
                # Befehl muss nachher herausgenommen werden
                #print(abs(hand_zwei-hand_eins))

                # Phase 1: Hände müssen zusammengehen
                # davor muss überprüft werden ob bereits der erste Teil der Abbruchbedinung schon kurz davor erfüllt wurde --> Überprüfung des Attributs self.abbruchbedingung_erster_teil_erfüllt == 0 (=1 wäre erste Phase wurde schon ausgeführt)
                # gleichzeitig wird der Abstand der Hände überprüft --> wenn der Abstand kleiner als 0.2 ist --> Wert wurde experimentell ermittelt und kann und soll später noch angepasst werden
                # sodass Abstand geprüft werden kann wird die Betragsfunktion genutzt sodass kein negativer Abstand als Ergebnis herauskommt
                
                if self.abbruchbedingung_erster_teil_erfüllt == 0 and abs(hand_zwei - hand_eins) < 0.2:
                    # Phase 1 ist demnach erfüllt --> self.abbruchbedingung_erster_teil_erfüllt = 1
                    # diese Variable ("Zähler") wird auch dafür genutzt dass der Roboterarm in dieser Zwischenzeit keine Bewegungen macht wenn die Abbruchbedingung gemacht wird 
                    self.abbruchbedingung_erster_teil_erfüllt = 1

                    # Wichtig ist dass diese Abbruchfunktion nicht ewig dauern kann wenn Phase 1 erfolgreich abgeschlossen wurde
                    # Problem wäre dass Bewegung eins Hände zusammen irgendwann mal stattfand und dann später zufälligerweise die Bewegung nach außen was ein Programmabbruch nach sich ziehen würde
                    # Nutzung der Zeitfunktion

                    # Start der "Stoppuhr"
                    # time.time() Befehl greift direkt auf Systemuhr des Computers zu --> gibt vergangene Sekunden seit 1. Januar 1970 zurück
                    self.stoppuhr_abbruchbedingung = time.time()

                    # Befehl muss nachher herausgenommen werden
                    print(">>> PHASE 1 AKTIV: Hände sind zusammen!") # NEU

                # Phase 2: Hände müssen auseinandergehen

                # durch elif kann ein Druchgang für den Prozessor durch den nachfolgenden Block gespart werden 
                # denn wenn in dem Bild in dem die Hände gerade den Schwellenwert 0.2 unterschritten hatten können sie ja nicht gleichzeitig wieder auseinandergehen
                # das auseinandergehen wäre rein theoretisch erst im nächsten Bild möglich
                elif self.abbruchbedingung_erster_teil_erfüllt == 1:

                    # wichtig ist das Zeit geprüft wird --> wenn zu viel Zeit verstrichen ist wird angenommen dass die beiden Bewegungen nicht zusammengehören --> keine Abbruchgeste
                    # Zeit die verstrichen sein darf wird festgelegt auf 1 Sekunde (Wert kann und soll nachträglich noch angepasst werden)
                    # hierbei Nutzung der time.time() Funktion diese hat Zeit welche seit dem 1.Januar 1970 bis zu dem Zeitpunkt wenn der Befehl aufgerufen wird
                    # von dieser Zeit wird die "Stoppuhr Zeit" abgezogen --> diese wurde gespeichert als Phase 1 erfüllt wurde
                    if time.time() - self.stoppuhr_abbruchbedingung < 1.5:
                        # verstichene Zeit ist kürzer als eine Sekunde --> Nun kann geprüft werden ob die Hände auseinandergehen
                        # Nun wird Abstand zwischen Händen (landmark 9) berechnet --> experimentell ermittelt auf > 0.5 (Wert kann und soll später noch angepasst werden)
                        
                        if abs(hand_zwei-hand_eins) > 0.5:
                            # Hände sind auseinandergegangen --> Abbruchgeste wurde damit vollständig erfüllt --> Programm wird anschließend durch main beendet
                            # Nun muss der Zähler self.abbruchbedingung_erster_teil_erfüllt auf Null gesetzt werden
                            self.abbruchbedingung_erster_teil_erfüllt = 0
                            
                            # hier wird kein return gemacht --> erst ganz unten sodass ein return ganz am Ende alles zurückgibt
                            # dafür wird die nachfolgende Variable auf True gesetzt
                            abbruch_geste_erkannt = True


                    else:
                        # Zeit war länger als eine Sekunde --> Geste für Abbruchbedingung wird als nicht gewollt bestätigt
                        # "Gedächtnis" muss wieder auf den Startwert zurückgesetzt werden --> erst dann könnte Geste wieder gemacht werden
                        # Zeit (Stoppuhr) muss nicht resetet werden --> wird im if Block für erste Phase überschrieben 
                        self.abbruchbedingung_erster_teil_erfüllt = 0
                        # Debuggen: Befehl später löschen
                        print ("Timeout")

    
            # Steuerbefehle dürfen nur errechnet werden wenn die Abbruchgeste nicht druchgeführt wird
            # daher wird geprüft ob Phase 1 der Abbruchgeste noch nicht erfüllt wurde --> self.abbruchbedingung_erster_teil_erfüllt == 0
            if self.abbruchbedingung_erster_teil_erfüllt == 0:

            
                # bei der Steuerung ist von Nöten dass die beiden Händen genau unterschieden werden können. so darf die linke Hand nur durch bestimmte Bewegungen zu bestimmten Funktion während gleiches bei der rechten Hand gilt
                # media pipe gibt landmarks nicht gekoppelt an die jeweilige Hand aus, daher muss die jeweilige Hand mit den jeweiligen landmarks verbunden werden
                # hierbei Nutzung von zip
            
                for einzelne_hand, hand_etikett in zip(ergebnis.multi_hand_landmarks, ergebnis.multi_handedness):
                    # Durchlauf durch for Schleife ist:
                    # Python greift für einzelne Hand Element ergebnis.multi_hand_landmarks
                    # Gleichzeitig wird für hand_etikett das Element ergebnis.multi_handedness genommen

                    # für diesen Durchlauf hat for Schleife ein bestimmten "Ordner" --> eine Hand
                    # Zeichenmethode wird aufgerufen --> diese braucht drei Dinge:
                    # Worauf soll gemalt werden: aktuelles Bild
                    # Wo sollen Punkte gesetzt werden: steht im aktuellen Ordner --> einzelne Hand
                    # Wie sollen Punkte verbunden werden: mp_haende.HAND_CONNECTIONS
                    self.mp_zeichnen.draw_landmarks(aktuelles_bild, einzelne_hand, self.mp_haende.HAND_CONNECTIONS)


                    # durch die for Schleife und das Objekt welche hand kann abgerufen werden welche Hand in DIESEM Durchgang in den landmarks gerade berücksichtigt wird
                    # durch den Befehl wird im Objekt welches davor an welche_Hand übergeben wurde, wird spezielle "Schublade" geladen
                    # eine KI und auch Media Pipe kann nur Wahrscheinlichkeiten ausgeben. --> es gibt kein, das ist zu 100 Prozent die linke Hand
                    # Media Pipe sotiert die Liste mit der Wahrscheinlichkeiten so dass die höchste Wahrscheinlichkeit beim Index 0 zu finden ist
                    # durch das .label wird ein sauberer string ausgegeben --> "Left"/"Right"
                    welche_hand = hand_etikett.classification[0].label


                    # Nun kann durch einen if BLock für die jeweilige Hand gezielt die Steuerbefehle berechnet werden
                    if welche_hand =="Left":

                        # nun wird Totzonen Steuerung für die linke Hand implementiert
                        # wichtig zu beachten dass zwei verschiedene Systeme vorliegen
                        # Media Pipe welches Prozentwert zwischen 0.0 & 1.0 liefert
                        # während die Totzone Grenzen in Pixeln festgelegt sind
                        # --> Umrechnung von Pronzentwerten (Media Pipe) in Pixel



                        # um eine weitere Funktion mit der linken Hand möglich zu machen wird ein "Kupplungsprinzip" genutzt
                        # hierbei führt eine Faust und die vertikale Bewegung mit der Faust zu einer weiteren Motorsteuerung
                        # bevor die sonstigen Bewegungen (keine Faust) geprüft werden können muss geprüft werden ob eine Faust vorliegt
                        
                        # Vergleich dieser landmarks --> wenn mittlerer Knochen über der Kuppe ist --> Finger ist gebeugt (hier werden nur y Werte berücksichtigt)
                        # hierbei wird jeder Finger in einem if BLock berücksichtigt

                        # Zeigefinger (Index Finger)
                        if einzelne_hand.landmark[8].y > einzelne_hand.landmark[6].y:
                            finger_nicht_ausgestreckt_linke_hand += 1

                        # Mittelfinger (Middle Finger)
                        if einzelne_hand.landmark[12].y > einzelne_hand.landmark[10].y:
                            finger_nicht_ausgestreckt_linke_hand += 1

                        # Ringfinger (Ring Finger)
                        if einzelne_hand.landmark[16].y > einzelne_hand.landmark[14].y:
                            finger_nicht_ausgestreckt_linke_hand += 1

                        # kleiner Finger (Pinky)
                        if einzelne_hand.landmark[20].y > einzelne_hand.landmark[18].y:
                            finger_nicht_ausgestreckt_linke_hand += 1

                        # Daumen wird außen vorgelassen, da hier die horizontale Achse betrachtet werden müsste und dies zu unnötiger Rechenleistung führen würde
                        # denn eine Faust ist auch gut eindeutig erkennbar wenn der Daumen nicht betrachtet wird
                        
            
                        # zuerst werden die Koordinaten der Gelenke abgerufen
                        # hierbei werden verschiedene Koordinaten genutzt, da das versteifen auf eine Koordinate beim steuern zu längeren Wartezeiten führen würde
                        # so wäre landmark 9 zu weit in der Mitte und die Hand müsse eine viel größere Bewegung machen um die Totzonengrenzen zu überschreiten

                        # falls später in das aktuelle Bild weitere Informationen wie welches Gelenk gerade aktiv betrachtet wird, gezeichnet werden soll werden Ganzzahlen benötigt
                        # daher wird int() davorgehängt
                        # gleichzeitig erfolgt eine Umrechnung in Pixel in dem lediglich das Attribut (Höhe/Breite) mit dem Prozentwert multipliziert wird 
                        linke_hand_rechte_grenze = int(einzelne_hand.landmark[5].x * self.bild_breite)
                        linke_hand_linke_grenze = int(einzelne_hand.landmark[17].x * self.bild_breite)
                        linke_hand_obere_grenze = int(einzelne_hand.landmark[11].y * self.bild_hoehe)
                        linke_hand_untere_grenze = int(einzelne_hand.landmark[0].y * self.bild_hoehe)



                        # für eine Faust wäre die Variable finger_nicht_ausgestreckt_linke_hand = 4 (Daumen wird nicht betrachtet)
                        # wenn dem nicht so ist kann die normale Erkennung der Bewegung der linken Hand stattfinden
                        if finger_nicht_ausgestreckt_linke_hand != 4:


                            # nun werden die verschiedenen landmarks mit den Totzonengrenzen verglichen
                            # hierbei es ist notwendig keine == Operator zu nutzen, da beim Bilder machen der während der Bewegung der Hand nicht gewährleistet ist das diese genau die Totzonengrenze trifft
                            # --> Nutzung von > < Operatoren
                            # hierbei Beachtung des Koordinaten System von Media Pipe --> oben links 0,0
                        
                            # wenn Hand Totzone nach links verlässt wird Pixelwert kleiner als Totzonengrenze
                            if linke_hand_linke_grenze < self.e1_x:
                                # hier wird nun die Hilfsmethode geschwindigkeitsberechnung aufgerufen
                                # da diese Methode allgemein gehalten ist müssen die Objekte welche in die Berechnung miteinbezogen werden sollen übergeben werden
                                # Hier:
                                # Linke Hand bezogen auf die linke (linke_hand_linke_grenze) Grenze wird als Koordinate übergeben
                                # Totzonengrenze horizontal ist e1_x
                                # Bildschirmrand ist links
                                # max Servo Geschwindigkeit ist max_speed_servo_1

                                # die Hilfsmethode gibt lediglich positive Geschwindigkeitswerte aus:
                                # um dennoch ein Motor ansteuern zu können der in beide Richtung drehen kann wird eine Unterscheidung notwendig: Nutzung einer positiven Geschwindigkeit & negativen Geschwindigkeit
                                # nach links: positive Geschwindigkeit --> Geschwindigkeit braucht kein Vorzeichen switch
                                motor_1_steuerbefehl = self.hilfsmethode_geschwindigkeitsberechnung(linke_hand_linke_grenze, self.e1_x, self.bildschirmrand_links, self.max_speed_servo_1)



                            # bei der horizontalen Bewertung kann elif genutzt so kann ein if erspart werden --> wenn erstes if schon zutrifft wird elif Block nicht mehr beachtet
                            # wenn Hand Totzone nach rechts verlässt wird Pixelwert größer als Totzonengrenze
                            elif linke_hand_rechte_grenze > self.e2_x:
                                # hier wird nun die Hilfsmethode geschwindigkeitsberechnung aufgerufen
                                # da diese Methode allgemein gehalten ist müssen die Objekte welche in die Berechnung miteinbezogen werden sollen übergeben werden
                                # Hier:
                                # Linke Hand bezogen auf die rechte Grenze (links_hand_rechte_grenze) wird als Koordinate übergeben
                                # Totzonengrenze horizontal ist e2_x
                                # Bildschirmmitte bezogen auf die Breite
                                # max Servo Geschwindigkeit ist max_speed_servo_1

                                # die Hilfsmethode gibt lediglich positive Geschwindigkeitswerte aus:
                                # um dennoch ein Motor ansteuern zu können der in beide Richtung drehen kann wird eine Unterscheidung notwendig: Nutzung einer positiven Geschwindigkeit & negativen Geschwindigkeit
                                # nach rechts: negative Geschwindigkeit --> Geschwindigkeit braucht Vorzeichen switch
                                motor_1_steuerbefehl = -1* self.hilfsmethode_geschwindigkeitsberechnung(linke_hand_rechte_grenze, self.e2_x, self.bildschirm_mitte, self.max_speed_servo_1)



                            # wenn Hand Totzone nach oben verlässt wird Pixelwert kleiner als Totzonengrenze
                            if linke_hand_obere_grenze < self.e1_y:
                                # hier wird nun die Hilfsmethode geschwindigkeitsberechnung aufgerufen
                                # da diese Methode allgemein gehalten ist müssen die Objekte welche in die Berechnung miteinbezogen werden sollen übergeben werden
                                # Hier:
                                # Linke Hand bezogen auf die obere Grenze (linke_hand_obere_grenze) wird als Koordinate übergeben
                                # Totzonengrenze horizontal ist e1_y
                                # Bildschirmrand ist oben
                                # max Servo Geschwindigkeit ist max_speed_servo_2

                                # die Hilfsmethode gibt lediglich positive Geschwindigkeitswerte aus:
                                # um dennoch ein Motor ansteuern zu können der in beide Richtung drehen kann wird eine Unterscheidung notwendig: Nutzung einer positiven Geschwindigkeit & negativen Geschwindigkeit
                                # nach oben: positive Geschwindigkeit --> Geschwindigkeit braucht keinen Vorzeichen switch
                                motor_2_steuerbefehl = self.hilfsmethode_geschwindigkeitsberechnung(linke_hand_obere_grenze, self.e1_y, self.bildschirmrand_oben, self.max_speed_servo_2)


                        
                            # bei der vertikalen Bewertung kann elif genutzt so kann ein if erspart werden --> wenn erstes if schon zutrifft wird elif Block nicht mehr beachtet
                            # wenn Hand Totzone nach unten verlässt wird Pixelwert größer als Totzonengrenze
                            elif linke_hand_untere_grenze > self.e2_y:
                                # hier wird nun die Hilfsmethode geschwindigkeitsberechnung aufgerufen
                                # da diese Methode allgemein gehalten ist müssen die Objekte welche in die Berechnung miteinbezogen werden sollen übergeben werden
                                # Hier:
                                # Linke Hand bezogen auf die untere Grenze (linke_hand_untere_grenze) wird als Koordinate übergeben
                                # Totzonengrenze horizontal ist e2_y
                                # Bildschirmrand ist unten
                                # max Servo Geschwindigkeit ist max_speed_servo_2

                                # die Hilfsmethode gibt lediglich positive Geschwindigkeitswerte aus:
                                # um dennoch ein Motor ansteuern zu können der in beide Richtung drehen kann wird eine Unterscheidung notwendig: Nutzung einer positiven Geschwindigkeit & negativen Geschwindigkeit
                                # nach unten: negative Geschwindigkeit --> Geschwindigkeit braucht Vorzeichen switch
                                motor_2_steuerbefehl = -1 * self.hilfsmethode_geschwindigkeitsberechnung(linke_hand_untere_grenze, self.e2_y, self.bildschirmrand_unten, self.max_speed_servo_2)




                        else:  # wenn Faust gemacht wird ist "Kupplung" aktiviert

                            # nun ist nur noch vertikale Bewegung relevant
                            if linke_hand_obere_grenze < self.e1_y:
                                # hier wird nun die Hilfsmethode geschwindigkeitsberechnung aufgerufen
                                # da diese Methode allgemein gehalten ist müssen die Objekte welche in die Berechnung miteinbezogen werden sollen übergeben werden
                                # Hier:
                                # Linke Hand bezogen auf die untere Grenze (linke_hand_untere_grenze) wird als Koordinate übergeben
                                # Totzonengrenze horizontal ist e1_y
                                # Bildschirmrand ist oben
                                # max Servo Geschwindigkeit ist max_speed_servo_3

                                # die Hilfsmethode gibt lediglich positive Geschwindigkeitswerte aus:
                                # um dennoch ein Motor ansteuern zu können der in beide Richtung drehen kann wird eine Unterscheidung notwendig: Nutzung einer positiven Geschwindigkeit & negativen Geschwindigkeit
                                # nach oben: positive Geschwindigkeit --> Geschwindigkeit braucht kein Vorzeichen switch
                                motor_3_steuerbefehl = self.hilfsmethode_geschwindigkeitsberechnung(linke_hand_obere_grenze, self.e1_y, self.bildschirmrand_oben, self.max_speed_servo_3)



                            elif linke_hand_untere_grenze > self.e2_y:
                               # hier wird nun die Hilfsmethode geschwindigkeitsberechnung aufgerufen
                                # da diese Methode allgemein gehalten ist müssen die Objekte welche in die Berechnung miteinbezogen werden sollen übergeben werden
                                # Hier:
                                # Linke Hand bezogen auf die untere Grenze (linke_hand_untere_grenze) wird als Koordinate übergeben
                                # Totzonengrenze horizontal ist e2_y
                                # Bildschirmrand ist unten
                                # max Servo Geschwindigkeit ist max_speed_servo_3

                                # die Hilfsmethode gibt lediglich positive Geschwindigkeitswerte aus:
                                # um dennoch ein Motor ansteuern zu können der in beide Richtung drehen kann wird eine Unterscheidung notwendig: Nutzung einer positiven Geschwindigkeit & negativen Geschwindigkeit
                                # nach unten: negative Geschwindigkeit --> Geschwindigkeit braucht Vorzeichen switch
                                motor_3_steuerbefehl = -1 * self.hilfsmethode_geschwindigkeitsberechnung(linke_hand_untere_grenze, self.e2_y, self.bildschirmrand_unten, self.max_speed_servo_3)





                    
                    # durch die for schleife wird in jedem Durchgang eine Hand ausgewählt und nur diese wird dann in den if Blöcken geachtet
                    # daher kann nie gleichzeitig die rechte und linke Hand im System in der for schleife sein
                    # daher wird elif verwendet --> so kann Rechenleistung gesparrt werden wenn die rechte Hand im System ist

                    elif welche_hand =="Right":
                        # in diesem Block wird vom Grundprinzip analog vorgegangen wie im if Block der rechten Hand


                        # nun wird Totzonen Steuerung für die rechte Hand implementiert
                        # wichtig zu beachten dass zwei verschiedene Systeme vorliegen
                        # Media Pipe welches Prozentwert zwischen 0.0 & 1.0 liefert
                        # während die Totzone Grenzen in Pixeln festgelegt sind
                        # --> Umrechnung von Pronzentwerten (Media Pipe) in Pixel


                        # um eine weitere Funktion mit der rechten Hand möglich zu machen wird ein "Kupplungsprinzip" genutzt
                        # hierbei führt eine Faust zur Schließung des Greifarms
                        # bevor die sonstigen Bewegungen (keine Faust) geprüft werden können muss geprüft werden ob eine Faust vorliegt
                        
                        # Vergleich dieser landmarks --> wenn mittlerer Knochen über der Kuppe ist --> Finger ist gebeugt (hier werden nur y Werte berücksichtigt)
                        # hierbei wird jeder Finger in einem if BLock berücksichtigt

                        # Zeigefinger (Index Finger)
                        if einzelne_hand.landmark[8].y > einzelne_hand.landmark[6].y:
                            finger_nicht_ausgestreckt_rechte_hand += 1

                        # Mittelfinger (Middle Finger)
                        if einzelne_hand.landmark[12].y > einzelne_hand.landmark[10].y:
                            finger_nicht_ausgestreckt_rechte_hand += 1

                        # Ringfinger (Ring Finger)
                        if einzelne_hand.landmark[16].y > einzelne_hand.landmark[14].y:
                            finger_nicht_ausgestreckt_rechte_hand += 1

                        # kleiner Finger (Pinky)
                        if einzelne_hand.landmark[20].y > einzelne_hand.landmark[18].y:
                            finger_nicht_ausgestreckt_rechte_hand += 1

                        # Daumen wird außen vorgelassen, da hier die horizontale Achse betrachtet werden müsste und dies zu unnötiger Rechenleistung führen würde
                        # denn eine Faust ist auch gut eindeutig erkennbar wenn der Daumen nicht betrachtet wird
                        

                        # zuerst werden die Koordinaten der Gelenke abgerufen
                        # hierbei werden verschiedene Koordinaten genutzt, da das versteifen auf eine Koordinate beim steuern zu längeren Warteizeiten führen würde
                        # so wäre landmark 9 zu weit in der Mitte und die Hand müsse eine viel größere Bewegung machen um die Totzonengrenzen zu überschreiten

                        # falls später in das aktuelle Bild weitere Informationen wie welches Gelenk gerade aktiv betrachtet wird, gezeichnet werden soll werden Ganzzahlen benötigt
                        # daher wird int() davorgehängt
                        # gleichzeitig erfolgt eine Umrechnung in Pixel in dem lediglich das Attribut (Höhe/Breite) mit dem Prozentwert multipliziert wird

                        rechte_hand_linke_Grenze = int(einzelne_hand.landmark[5].x * self.bild_breite)
                        rechte_hand_rechte_Grenze = int(einzelne_hand.landmark[17].x * self.bild_breite)
                        rechte_hand_obere_Grenze = int(einzelne_hand.landmark[11].y * self.bild_hoehe)
                        rechte_hand_untere_Grenze = int(einzelne_hand.landmark[0].y * self.bild_hoehe)


                        if finger_nicht_ausgestreckt_rechte_hand != 4:


                            # nun werden die verschiedenen landmarks mit den Totzonengrenzen verglichen
                            # hierbei es ist notwendig keine == Operator zu nutzen, da beim Bilder machen der während der Bewegung der Hand nicht gewährleistet ist das diese genau die Totzonengrenze trifft
                            # --> Nutzung von > < Operatoren
                            # hierbei Beachtung des Koordinaten System von Media Pipe --> oben links 0,0

                            # wenn Hand Totzone nach links verlässt wird Pixelwert kleiner als Totzonengrenze
                            if rechte_hand_linke_Grenze < self.e4_x:
                                # hier wird nun die Hilfsmethode geschwindigkeitsberechnung aufgerufen
                                # da diese Methode allgemein gehalten ist müssen die Objekte welche in die Berechnung miteinbezogen werden sollen übergeben werden
                                # Hier:
                                # Rechte Hand bezogen auf die linke (rechte_hand_linke_grenze) Grenze wird als Koordinate übergeben
                                # Totzonengrenze horizontal ist e4_x
                                # Bildschirmmitte bezogen auf die Breite
                                # max Servo Geschwindigkeit ist max_speed_servo_4

                                # die Hilfsmethode gibt lediglich positive Geschwindigkeitswerte aus:
                                # um dennoch ein Motor ansteuern zu können der in beide Richtung drehen kann wird eine Unterscheidung notwendig: Nutzung einer positiven Geschwindigkeit & negativen Geschwindigkeit
                                # nach links: positive Geschwindigkeit --> Geschwindigkeit braucht kein Vorzeichen switch
                                motor_4_steuerbefehl = self.hilfsmethode_geschwindigkeitsberechnung(rechte_hand_linke_Grenze, self.e4_x, self.bildschirm_mitte, self.max_speed_servo_4)



                            # bei der horizontalen Bewertung kann elif genutzt so kann ein if erspart werden --> wenn erstes if schon zutrifft wird elif Block nicht mehr beachtet
                            # wenn Hand Totzone nach rechts verlässt wird Pixelwert größer als Totzonengrenze
                            elif rechte_hand_rechte_Grenze > self.e3_x:
                                # hier wird nun die Hilfsmethode geschwindigkeitsberechnung aufgerufen
                                # da diese Methode allgemein gehalten ist müssen die Objekte welche in die Berechnung miteinbezogen werden sollen übergeben werden
                                # Hier:
                                # Rechte Hand bezogen auf die rechte Grenze (Rechte_hand_rechte_grenze) wird als Koordinate übergeben
                                # Totzonengrenze horizontal ist e3_x
                                # Bildschirmrand rechts
                                # max Servo Geschwindigkeit ist max_speed_servo_4

                                # die Hilfsmethode gibt lediglich positive Geschwindigkeitswerte aus:
                                # um dennoch ein Motor ansteuern zu können der in beide Richtung drehen kann wird eine Unterscheidung notwendig: Nutzung einer positiven Geschwindigkeit & negativen Geschwindigkeit
                                # nach rechts: negative Geschwindigkeit --> Geschwindigkeit braucht Vorzeichen switch
                                motor_4_steuerbefehl = -1* self.hilfsmethode_geschwindigkeitsberechnung(rechte_hand_rechte_Grenze, self.e3_x, self.bildschirmrand_rechts, self.max_speed_servo_4)




                            # wenn Hand Totzone nach oben verlässt wird Pixelwert kleiner als Totzonengrenze
                            if rechte_hand_obere_Grenze < self.e3_y:
                                # hier wird nun die Hilfsmethode geschwindigkeitsberechnung aufgerufen
                                # da diese Methode allgemein gehalten ist müssen die Objekte welche in die Berechnung miteinbezogen werden sollen übergeben werden
                                # Hier:
                                # Rechte Hand bezogen auf die obere Grenze (rechte_hand_obere_grenze) wird als Koordinate übergeben
                                # Totzonengrenze horizontal ist e3_y
                                # Bildschirmrand ist oben
                                # max Servo Geschwindigkeit ist max_speed_servo_5

                                # die Hilfsmethode gibt lediglich positive Geschwindigkeitswerte aus:
                                # um dennoch ein Motor ansteuern zu können der in beide Richtung drehen kann wird eine Unterscheidung notwendig: Nutzung einer positiven Geschwindigkeit & negativen Geschwindigkeit
                                # nach oben: positive Geschwindigkeit --> Geschwindigkeit braucht keinen Vorzeichen switch
                                motor_5_steuerbefehl = self.hilfsmethode_geschwindigkeitsberechnung(rechte_hand_obere_Grenze, self.e3_y, self.bildschirmrand_oben, self.max_speed_servo_5)


                            
                            # bei der vertikalen Bewertung kann elif genutzt so kann ein if erspart werden --> wenn erstes if schon zutrifft wird elif Block nicht mehr beachtet
                            # wenn Hand Totzone nach unten verlässt wird Pixelwert größer als Totzonengrenze
                            elif rechte_hand_untere_Grenze > self.e4_y:
                                # hier wird nun die Hilfsmethode geschwindigkeitsberechnung aufgerufen
                                # da diese Methode allgemein gehalten ist müssen die Objekte welche in die Berechnung miteinbezogen werden sollen übergeben werden
                                # Hier:
                                # Rechte Hand bezogen auf die untere Grenze (rechte_hand_untere_grenze) wird als Koordinate übergeben
                                # Totzonengrenze horizontal ist e4_y
                                # Bildschirmrand ist unten
                                # max Servo Geschwindigkeit ist max_speed_servo_5

                                # die Hilfsmethode gibt lediglich positive Geschwindigkeitswerte aus:
                                # um dennoch ein Motor ansteuern zu können der in beide Richtung drehen kann wird eine Unterscheidung notwendig: Nutzung einer positiven Geschwindigkeit & negativen Geschwindigkeit
                                # nach unten: negative Geschwindigkeit --> Geschwindigkeit braucht Vorzeichen switch
                                motor_5_steuerbefehl = -1 * self.hilfsmethode_geschwindigkeitsberechnung(rechte_hand_untere_Grenze, self.e4_y, self.bildschirmrand_unten, self.max_speed_servo_5)


                            # das Gedächtnis für die Faust muss aktualisiert werden
                            self.letztes_bild_war_faust = False



                        else:   # wenn Faust gemacht wurde wird Greifarm geschlossen
                            # hierfür wird keine Geschwindigkeitsberechnung vorgenommen. Der Greifarm wird mit seiner vollen Geschwindigkeit geschlossen
                            # da die rechte Hand beim Faust zu machen den Greifarm schließen soll aber die Faust nicht durchgehen gehalten werden kann wird ein "Gedächtnis" benötigt

                            # wenn nun 4 Finger gezählt sind und im letzten Bild keine Hand zu sehen war darf umgeschaltet werden
                            if self.letztes_bild_war_faust == False:
                                # Zustand muss umgedreht werden --> Nutzung von not --> aus False wird True, aus True wird False
                                self.greifer_ist_zu = not self.greifer_ist_zu


                            # "Gedächtnis" muss aktualisiert werden
                            self.letztes_bild_war_faust = True

                            # abschließend muss der Steuerbefehl berechnet werden
                            if self.greifer_ist_zu == True:
                                # Greifer soll zugehen --> Fahre rückwärts (Minus Vorzeichen)
                                motor_6_steuerbefehl = -1* self.max_speed_servo_6 

                            else:
                                # Greifer soll aufgehen --> Fahre vorwärts
                                motor_6_steuerbefehl = self.max_speed_servo_6                    
                                

                        


        # am ende werden die Steuerbefehle in Form einer Liste zurückgegeben
        # da zu Beginn die Motorsteuerbefehle alle auf Null gesetzt (Ausnahme Servo 6) sind wird bei keiner Handerkennung ein Steuerbefehl mit nur Nullen zurückgemeldet ohne das ein Fehler entsteht
        steuerbefehle = [motor_1_steuerbefehl, motor_2_steuerbefehl, motor_5_steuerbefehl, motor_4_steuerbefehl, motor_3_steuerbefehl, motor_6_steuerbefehl]
        


        return aktuelles_bild, steuerbefehle, abbruch_geste_erkannt
    

    def hilfsmethode_geschwindigkeitsberechnung (self, pixel_koordinate_hand, totzonen_grenze, bildschirm_rand, max_motor_geschwindigkeit):
        # diese Hilfsmethode erledigt für die Methode Berechne Steuerbefehle die Geschwindigkeitsberechnung für die Motoren
        # Idee:
        # wenn die Hand die Totzone verlässt soll in Abhängigkeit vom Abstand von der Grenze eine Geschwindigkeit berechnet werden
        # hierbei wird aber nicht einfach irgendein Wert wie der Abstand zur Grenze ausgegeben sondern es wird der Abstand der Grenze zum Bildschirmrand genutzt und dieser wird durch den Handabstand geteilt und mit einem festgelegten Wert multipliziert
        # dadurch wird ein Wert (bestimmter Wert als bsp. auf 10 festgelegt) zwischen 0 und 10 erreicht
        # durch int() wird aus diesem Wert nun eine Ganzzahl
        # das bringt zwei Vorteile mit sich:
        # der Arduino bekommt als Steuerbefehl nur Werte zwischen 0 und 10 und nur als Ganzzahl, anstatt lange große Zahlen die dieser dann selbst umrechnen müsste
        # es wird eine erste Filterung der Werte vorgenommen. Durch das Zittern der Hand bzw. andere Einflüsse wie die Lichtverhältnisse findet ein "zittern" der landmarks statt (die Werte sind nicht so eindeutig)
        # durch die GAnzzahl wird nun radikal der Wert zwischen 5.00045 und 5.056 einfach zu einer 5 gemacht und der Arduino bekommt viel klarere Befehle
        #
        # die Werte für die hilfsmethode werden in Abhängigkeit der benötigten Motoren Steuerbefehle übergeben
        # die max. Geschwindigkeit ist als Attribut in der Klasse gespeichert
        # wird aber sodass die Geschwindigkeit der einzelnen Motoren variiert werden könnte immer beim Aufruf der Hilfsmethode übergeben

        
        # zuerst wird Abstand zwischen Totzonengrenze und Bildschirmrand bzw. Bildschirmmitte berechnet
        # Nutzung Betrag sodass keine negative Werte entstehen
        abstand_totzonengrenze_bildschirmrand =abs(totzonen_grenze - bildschirm_rand)

        # sodass im Fall der Entstehung einer Null als Ergebnis in der oberen Rechnung es nachfolgend zu keinem Fehler führt, wird der Wert vor seiner nachfolgenden Nutzung geprüft
        if abstand_totzonengrenze_bildschirmrand == 0:
            # wenn das der Fall wäre wird zur Sicherheit einfach Null als Geschwindigkeit zurückgegeben
            return 0

        # analog dazu muss der Abstand der landmark (in Pixel schon umgerechnet) von der Totzone berechnet werden
        abstand_hand_totzone = abs(totzonen_grenze - pixel_koordinate_hand)

        # Nun werden die beiden Abstände geteilt --> das ergibt einen Prozentwert und dieser wird mit der als Attribut festgelegte Geschwindigkeit multipliziert
        # wenn die landmark völlig am Bildschirmrand ist wäre der Prozentwert 1 --> * 10 --> volle Geschwindigkeit
        # Int --> Ganzzahl
        prozentwert = abstand_hand_totzone / abstand_totzonengrenze_bildschirmrand

        # dadurch das Media Pipe nicht wie if else strikt funktioniert kann es zu Ausreißern bzw. fehlerhaften Werten kommen, sodass die Motoren keine ungültigen Geschwindigkeitswerte bekommen wird durch min() max() das ganze begrenzt
        # Achtung min() --> Obergrenze, max() --> Untergrenze
        # Funktionsweise: Funktion min(Zahl_A, Zahl_B) --> vergleicht beide Zahlen und gibt immer die kleinere zurück
        # 1.0 --> 100 % wäre der maximale Abstand den die Hand von der Totzonengrenze in der Theorie haben kann
        # wenn der Wert größer als 1.0 ist wird einfach 1.0 zurückgegeben
        prozentwert = min(prozentwert, 1.0)

        # 0.0 --> 0% ist der minimalste Abstand den die Hand von der Totzonengrenze in der Theorie haben kann
        prozentwert = max(0.0,prozentwert)

        # Prozentwert ist durch die beiden Funktionen genau zwischen 0.0 und 1.0, jetzt kann max geschwindigkeit multipliziert werden um den jeweiligen Geschwindigkeitswert in Abhängigkeit der landmark Position zu erhalten
        geschwindigkeit = int(prozentwert * max_motor_geschwindigkeit)

        # am Ende wird errechnete Geschwindigkeit zurückgegeben
        return geschwindigkeit



    


    def zeige_bild_und_pruefe_abbruch(self, aktuelles_bild):


        cv2.putText(aktuelles_bild, "  MEDIAPIPE: TRACKING AKTIV | Modus: Teleoperation", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(aktuelles_bild, "  KOLLISIONSVERMEIDUNG: INAKTIV", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Befehl öffnet grafisches Fenster
        cv2.imshow("Test", aktuelles_bild)

        # Tastatur prüfen (q zum Abbrechen)
        if cv2.waitKey(1) == ord('q'):
            return True
        else:
            return False
        


    def zeige_abspann(self):
        # durch die Abbruchgeste ist keine ausreichende Nutzer experience gewährleistet da der Stream abbrupt abbricht
        # da dies in keinster der Anspruch sein kann wird diese Methode genutzt
        # durch eine Schleife wird das Kamerabild analog wie in der Methode berechne steuerbefehle aufgenommen und dann aber direkt zurückgegeben
        # dadurch wird keine unnötige Rechenleistung genutzt wie wenn nochmals die Methode berechne steuerbefehle gestartet werden würde

        # Nutzung des Zeitstempels
        startzeit = time.time()

        # while Schleife soll 0.6 Sekunden laufen --> time.time() (zeit zum jetzigen Zeitpunkt) - startzeit (Zeitstempel) < 1.0
        while (time.time() - startzeit) < 2.0: 

            # Befehl um die Kamera auszulösen --> kamera.read()
            # Befehl kann durch boolsche Variable zurückmelden ob Bild erfolgreich gemacht wurde
            # Bild selbst (aktuelles Bild) --> "Riesige Excel Tabelle mit Farbwerten"
            erfolg, aktuelles_bild = self.kamera.read()

            if not erfolg: # wenn Bild nicht erfolgreich muss Funktion direkt abgebrochen werden
                print("Warnung: Kamerafehler. Breche Bildauswertung ab.")
                # bei Kamerafehler --> beenden der while Schleife
                break

            # spiegeln des Kamerabildes sodass der Nutzer bei der Hand nicht umdenken muss --> wenn er die rechte Hand hebt geht auch auf dem Bildschirm von ihm aus gesehen die rechte Hand hoch
            # Bild horizontal spiegeln --> cv.flip(aktuelles_bild,1) --> 1 = horizontal, 0 = vertikal
            aktuelles_bild = cv2.flip(aktuelles_bild, 1)


            #Befehl öffnet grafisches Fenster
            cv2.imshow("Test", aktuelles_bild)

            # es ist notwendig um Bild auf zu Monitor zu bringen, dass Code gezwungen wird kurz stehen zu bleiben --> (1) --> 1 millisekunde
            cv2.waitKey(1)
            


    def zeichne_totzone(self,aktuelles_bild):
        # Parameter bzw. Koordinaten sind in init definiert

        # um Rechtecke zu zeichnen wird rectangle(aktuelles_bild,(Linke_Ecke_oben_x, Linke_Ecke_unten_y), (Rechte_Ecke_unten_x, Rechte_Ecke_unten_y),(BGR Werte),Linienstärke) Funktion genutzt
        # es Funktion berechnet die direkten Linien dazwischen und überschreibt Pixel des eigtl Bilds mit den angegeben BGR Werten
        cv2.rectangle(aktuelles_bild, (self.e1_x, self.e1_y),(self.e2_x,self.e2_y),(255,255,0),2)

        cv2.rectangle(aktuelles_bild, (self.e3_x,self.e3_y),(self.e4_x,self.e4_y),(255,255,0),2)


        # zurückgegeben wird das aktuelle BIld in welchem die Totzone Kästen eingezeichnet sind
        return aktuelles_bild





    def verbindung_kamera_trennen(self):
        # Hardware freigeben
        self.kamera.release()
        # Arbeitsspeicher leeren
        cv2.destroyAllWindows()


 


