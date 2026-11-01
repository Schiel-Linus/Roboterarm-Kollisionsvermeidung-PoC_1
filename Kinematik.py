"""
=========================================================
PROJEKT:        Roboterarm mit prädiktiver Kollisionsvermeidung
DATEI:          Kinematik_Milestone_5.py
BESCHREIBUNG:   Bewegungssteuerung und Bahnplanung. Übersetzt 
                Geschwindigkeitsbefehle (dt-basiert) in Winkel 
                und sorgt über lineare Rampen sowie S-Rampen für 
                ruckelfreie und sanfte Servobewegungen.
---------------------------------------------------------
ENTWICKLER:     Linus Schiel
VERSION:        v1.0.0 (Proof of Concept 1)
=========================================================
"""



# Problemstellung:
# der digital twin und der physische Roboterarm müssen in der exakt identischen Position sein
# jedoch besteht die Möglichkeit dass der physiche Roboterarm in einer variablen Position steht die erheblich unterschiedlich zum twin ist
# nun wäre eine einfache Lösung dem Arduino bzw. den einzelnen Servo Motoren eine Start Sequenz in Form von Winkel zu schicken
# da die zum jetzigen Zeitpunkt vorliegenden Servos aber keine Winkelposition zurückmelden können ist für das Programm bzw. für den Bediener nie klar in welcher Position der Arm steht
# wenn nun eine Homing Sequenz kommen würde die stark abweichend von der jetzigen Position wäre, würden die Servo mit voller Geschwindigkeit anfahren und abrupt abbremsen
# die Folge: extreme Kräfte wirken auf die einzelnene Komponenten und zusätzlich kein zufriedenstellendes Nutzer Erlebnis
# 
# Lösung:
# durch CAD und späteren 3D Druck werden Aufnahmen/Halterung/Fixierung gedruckt in welchen der Roboterarm am Ende abgelegt wird
# am Anfang wird eine Startsequenz in Form von Winkel an die einzelnen Servos gesendet
# da Servos jedoch schon in dieser Position liegen müssen die Servo nicht nachsteuern
# in der Praxis kann es dennoch zu leichten Bewegungen kommen da die einzelnen Komponenten mit erheblichen Toleranzen belastet sein können
# jedoch werden dennoch keine extremen Bewegungen hervorgerufen
# nach der Homing Sequenz erhalten die Motoren eine vorgefertigte Winkelsequenz wodurch sich der Arm sehr smooth aufklappt und in die richtige Startsequenz fährt
# für diese Winkelsequenz wird die Geschwindigkeit einer S-Ramp angenähert



import time

class Kinematik:
    
    def __init__(self):
        # Achtung: Servo_6 (Greifarm) und dessen Steuerung besitzt eine Besonderheit: es gibt nur Greifarm auf und Greifarm zu
        # in Klasse Vision bedeutet es gibt nur ein An oder Aus Signal --> 10 oder 0 --> daher muss der 6. Servo in dieser Kinematik komplett seperat betrachtet werden

        # Eine weitere Besonderheit besteht darin dass hier mit einem open Loop System gearbeitet wird, daher ist der jeweilige Winkel der Servo Motoren aufgrund der vorliegenden Servo Motoren nicht bekannt
        # In naher Zukunft wäre es eine mögliche Verbesserung die Hardware upzugraden um so das System noch robuster zu gestalten
        # Allerdings ist es zum jetzigen Zeitpunkt so programmiert dass das open Loop System auch mit dem digital twin verknüpft ist


        # in der Klasse werden die Winkel der Motoren gespeichert

        
        # Start Koordinaten die festlegen in welcher Ausgangsposition der Roboterarm in der Ruhelage war
        self.winkel_ruhelage_roboterarm = [0, 0, 0, 0, 0, 0]


        # die aktuellen Winkel der Servos --> zu Beginn stehen diese auf den Start Koordinaten welche bei den Attributen der S Rampe definiert sind --> daher wird zu Beginn Liste kopiert
        self.aktuelle_winkel = self.winkel_ruhelage_roboterarm.copy()
        
        # die aktuelle Geschwindigkeiten der Servos --> zu Beginn ist die Geschiwndigkeit bei den Motoren 0
        self.aktuelle_geschwindigkeiten = [0,0,0,0,0,0]
        
        # da die Servomotoren nur von 0 bis 270 Grad fahren können werden limits benötigt
        # allerdings kann es notwendig sein diese Grenzen noch weiter anzupassen (baulich bedingt)
        self.min_winkel_limits = [0,0,0,0,0,0] 
        self.max_winkel_limits = [270,270,270,270,270,270] 

        

        # Attribute welche für die lineare Rampe benötigt werden

        # Beschleunigungsrate der Servomotoren
        # 0.5 ist lediglich ein Platzhalter
        # Servomotoren können/sollten nicht mit voller Geschwindigkeit direkt "losfahren" um starkes ruckeln zu vermeiden und sanftes anfahren zu gewährleisten
        self.beschleunigungssrate = [180.0, 180.0, 180.0, 180.0, 180.0, 0.5]

        # Geschwindigkeitsfaktor der Servomotoren
        # dieser Wert ist die Übersetzung in eine Grad Zahl --> der maximale Wert wäre dann bspw. 10 grad pro Sekunde bei einer Geschwindigkeitsrückmeldung der Klasse Vision von 10
        self.max_geschwindigkeit = [9.0, 9.0, 9.0, 9.0, 9.0, 1]

        # Zeit
        # time.time() Befehl greift direkt auf Systemuhr des Computers zu --> gibt vergangene Sekunden seit 1. Januar 1970 zurück
        self.letzte_zeit = time.time()


        # Attribute welche für die S Rampe benötigt werden

        # für die Kamera Steuerung ist eine reine lineare Geschwindigkeitsrampe in Gebrauch
        # denn für eine S Rampe werden immer Ziel Koordinaten gebraucht werden --> jedoch sind Zielkoordinaten bei einer Kamerasteuerung nicht möglich wenn mit einem Kamerasystem bzw. einer Joystick-Steuerung gearbeitet wird
        # allerdings wird der Roboterarm aus der Ruhelage aufgeklappt und wieder eingeklappt in diesen Situationen sind die Zielkoordinaten feststehend und eindeutig daher kann eine S Rampe modelliert werden
        # diese S Rampe bzw. die Winkelkoordinaten wird in einer Hilfsmethode modelliert
        # dadurch ist eine hohe Flexibilität möglich da nun durch jede beliebige weitere Methode diese Hilfsmethode aufgerufen werden kann
        
        # weitergehend wird in dieser Klasse eine Hierarchiestruktur implementiert welche steuert welches System gerade aktiv ist, bspw. kann so gesteuert werden ob die Kamera für die Steurung jetzt zuständig ist oder ein anderes System
        # daher ist das System so aufgebaut dass leicht ein weiteres System hinzugefügt werden kann. Beispiele dafür könnten die Steuerung über inverse Kinematik sein

        # Notwendig dafür ist eine Variable welchen den Zustand speichert der gerade aktiv ist
        # bei den verschiedenen Modis werden so bestimmte strings hinterlegt bspw. wenn der Roboterarm sich aufklappt wird aufklappen hinterlegt
        self.aktueller_modus = "AUFKLAPPEN"
        

        # Modus Aufklappen:
        # für die später genutzten Methoden wird eine Startzeit für das Aufklappen benötigt
        self.startzeit_aufklappen = time.time()


        # Ziel Koordinaten für die neue Position des Arm in welchem dann die Kamera die Steuerung übernimmt
        self.winkel_ziel_position_roboterarm = [90, 90, 90, 90, 90, 90]

        # Ziel Dauer wie lange ausklappen dauern soll (jetzt 4 Sekunden)
        self.dauer_aufklapp_vorgang = 4

        # Modus Einklappen:
        # für die spätere genutzte Methode wird eine Liste der Winkel vor dem Einklappen benötigt
        # zu Beginn sind dort nur 0 als Platzhalter später in update zyklus Methode wenn ein Abbruch erkannt ist werden die aktuelle Winkel in diese Methode kopiert
        self.winkel_vor_einklappen = [0,0,0,0,0,0]

        # auch wird eine Startzeit benötigt allerdings wird diese erst dann aktualisiert wenn update zyklus den Abbruch erkannt hat daher ist auch hier ein Platzhalter
        self.startzeit_einklappen = 0

        # Ziel Dauer wie lange einklappen dauern soll (jetzt 4 Sekunden)
        self.dauer_einklapp_vorgang = 4

        
        





    # Ablauf dieser Klasse zusammengefasst
    # die Methode update zyklus läuft in der main in einer while schleife in main
    #
    # 1. Eintritt in den Manager
    # bei jedem Durchlauf ruft die Hauptschleife die update_zyklus Methode auf --> der "Manager" prüft den Status
    # da Programm gerade erst gestartet wurde steht dort "Aufklappen" --> daher wird sofort die methode modus aufklappen aufgerufen
    # 
    # 2. Die Stoppuhr läuft (Tau wächst)
    # Durchlauf 1 (z.B. nach 0,1 Sekunden): Methode misst die Zeit --> Es sind 0,1 Sekunden vergangen --> Tau ist demnach 0,1 / 4 = 0,025
    # Die Methode übergibt diese 2.5% an die S Rampe --> die Mathematik das Polynom berechnet wo sich der Arm bei 2.5% der Zeit befinden muss (sehr nahe am Startwinkel, da die Beschleunigung noch gering ist)
    # Der Arduino bekommt den String, der Arm zuckt einen Millimeter
    # die methode ist fertig und das Programm springt sofort zurück an den Anfang der while Schleife 
    # 
    # 3. Die kontinuierliche Aktualisierung
    # Durchlauf 100 (z.B. nach 2.0 Sekunden) --> Schleife läuft immer weiter --> wieder prüft Manager ob Aufklapp Modus aktiv ist
    # Stoppuhr = 2.0 Sekunden --> tau 2.0 / 4.0 = 0.5
    # S Rampe spuckt genau den Mittelwert zwischen Start und Zielwinkel aus --> Arm bewegt sich mit maximaler Geschwindigkeit durch die Mitte seiner Flugbahn
    # Wieder geht string an den Arduino. Wieder beginnt Schleife von vorn
    # 
    # 4. Der automatische Übergang (Schalter fällt)
    # Durchlauf 200 (z.B. nach 4,01 Sekunden) --> System misst 4.01 Sekunden
    # tau wird berechnet und überschreitet die magische Grenze von 1.0
    # S RAmpen Hilfsmethode greift sofort ein und schneidet tau auf 1.0 (min/max) --> dadurch ist sichergestellt das Motor exakt auf den Zielwinkel fährt und nicht darüber hinaus
    # Jetzt schlägt if (tau >= 1) zu --> self.aktueller modus wird auf Teleoperation geändert
    # Der String geht mit der perfekten Zielposition an den Arduino --> Arm steht still in seiner Aufklapp Position
    # 
    # 5. Der neue Modus (Teleoperation)
    # Durchlauf 201 --> die Hauptschleife ruft wieder Manager auf dieser prüft und ruft durch das Attribut Teleoperation den modus teleoperation auf
    # ab jetzt wird in jedem Takt die Handkoordinate der Kamera ausgewertet (Klasse Vision) und über dt wird eine Skalierung in Form einer linearen Rampe vorgenommen
    # Arduino erhält wieder in Form eines strings die Winkel 
    


    def update_zyklus(self, geschwindigkeitswerte_vision_klasse, abbruchgeste_erkannt):
        # die update zyklus Methode ist der Manager der Klasse und steuert in Abhängigkeit des Attributes aktueller Status die nachfolgenden Methoden modus aufklappen, modus teleoperation & modus einklappen

        jetzt = time.time()

        # zuerst muss die vergangene zwischen dem letzten und dem jetzigen Methoden durchlauf genommen werden
        dt = jetzt - self.letzte_zeit
            
        # jetzt muss die aktualisierte Zeit wieder als Attribut in der Klasse gespeichert/aktualisiert werden
        self.letzte_zeit = jetzt
        
        # dt wird aus zwei Gründen benötigt
        # 1.:
        # ein Servo kann nur Winkelkoordinaten nutzen, keine Geschwindigkeiten
        # um nun von einer berechneten Geschwindigkeit auf einen Winkel zu kommen muss der Zurückgelegte Weg berechnet werden
        # Zurückgelegter Weg = Geschwindigkeit * Zeit 
        # --> Servo erhält dann eine Winkelkoordinate in dem Durchlauf um die gewünschte Geschwindigkeit zu erreichen
        
        #2.:
        # while Schleifen laufen nie gleich schnell ab --> mal dauert es 10 ms für einen Durchlauf mal 40 ms weil Windows im HIntergrund zusätzlich etwas anderes macht
        # ist der PC schnell --> hämmert er in Sekunde 100 Befehle heraus --> Arm schnellt 100 Grad nach vorne
        # ist PC langsam --> schafft er nur 10 Befehle --> Arm kriecht nur mit 10 Grad nach vorne
        # --> Bewegung des Roboters wäre direkt vom Prozessor des PC abhängig --> ständiges zucken und stottern
        # Lösung: --> Nutzung von dt (Messung wie viel echte Zeit zwischen dem letzten Durchlauf vergangen ist)
        # Lief PC schnell (dt ist winzig) rechnet Formel einen winzigen Winkelschritt aus
        # Lief PC langsam (dt ist groß) rechnet Formel größeren Winkelschritt aus
        
        # Beispiel:
        # 1. Hand befiehlt 10 Grad pro Sekunde (weitere Faktoren werden bewusst außen vor gelassen)
        # Zeitschleife misst dt = 20 ms
        # --> Winkelschritt = 10°/s * 0.02s = 0.2°
        # --> Zielwinkel = 90.2°
        # 2. neues Wertepaar: 10°/s, 15 ms                                         
        #  --> Zielwinkel: 90.2° + 10°/s * 0.015s =  90.35
        # 3. neues Wertepaar: 10°/s, 15 ms
        # --> Zielwinkel: 90.35° + 10°/s * 0.015s = 90.5°
        # 
        # wenn jetzt aber die while Schleife länger gebraucht hätte nämlich anstatt zweimal 15ms einmal 30ms:
        # neues Wertepaar: 10°/s, 30 ms                     
        # --> Zeilwinkel = 90.2° + 10°/s * 0.03s = 90.5°
        # --> es kommt dementsprechned genau der gleiche Winkel heraus denn der Servo anfahren muss obwohl die while Schleife viel länger gebraucht hat
        
        # es ist allerdings auch logisch dass diese Zeiten nicht zu groß werden dürfen, weil der Servo sonst stoppt (weil Winkel schon erreicht ist) und wieder anfährt nach einem neuen Befehl was zu starkem ruckeln führt
        # jedoch sollte auch berücksichtigt werden wie ein Servo arbeitet:
        # ein Servo nutzt immer den neuesten Wert welchen er bekommt wenn es 90.2° er immer noch dort hin unterwegs ist und jetzt 80.4° kommt ändert er sofort die Richtung --> es gibt kein Stau von Winkelbefehlen
        


        # davor muss überprüft werden welcher Modus gerade aktiv ist
        if self.aktueller_modus == "AUFKLAPPEN":
            # nun wird Methode aufklappen aktiviert
            # das Ergebnis der Methode (die Winkel in Form eines strings) wird direkt zurückgegeben
            return self.modus_aufklappen()

        # wenn Modus Aufklappen beendet ist und Attribut aktueller Modus auf "Teleoperation" umgeschaltet wurde muss die andere Methode --> modus teleoperation gestartet werden
        elif self.aktueller_modus == "TELEOPERATION":

            # Allerdings muss bevor auf die Kamerasteuerung umgeschalten wird geprüft werden ob die Vision Klasse eine Abbruchgeste erkannt hat
            # Hierfür wird der übergeben Werte abbruchgeste_erkannt auf True überprüft (True --> bedeutet Abbruchgeste erkannt)
            if abbruchgeste_erkannt == True:
                # wenn abbruchgeste erkannt muss aktueller Modus umgeschaltet werden
                self.aktueller_modus = "EINKLAPPEN"

                # für die Methode Einklappen wird die Startzeit benötigt
                self.startzeit_einklappen = time.time()

                # gleichzeitig werden die aktuellen Winkel der einzelnen Motoren genau zum jetzigen Zeitpunkt gebraucht, dafür wird die Liste aktuelle Winkel kopiert
                self.winkel_vor_einklappen = self.aktuelle_winkel.copy()

                return self.modus_einklappen()

            else:
                # Methode teleoperation wird aktiviert, dafür müssen die Geschwindigkeitswerte der Vision Klasse übergeben werden sowie das dt
                # Ergebnis der Methode (die Winkel in Form eines strings) wird direkt zurückgegeben
                return self.modus_teleoperation(geschwindigkeitswerte_vision_klasse, dt)

        # wenn Attribut auf Einklappen geändert wurde muss die Einklappen Methode gestartet werden
        elif self.aktueller_modus == "EINKLAPPEN":
            # nun wird Methode einklappen aktiviert
            # das Ergebnis der Methode (die Winkel in Form eines strings) wird direkt zurückgegeben
            return self.modus_einklappen()
            

         

    def modus_aufklappen(self):
        # Diese Methode steuert das Aufklappen des Roboterarms

        # zuerst muss die bis zum jetzigen Zeitpunkt verstrichene Zeit augelesen werden
        verstrichene_zeit = time.time() - self.startzeit_aufklappen

        # tau --> verstrichene Zeit durch die festgelegte Aufklappdauer
        tau = verstrichene_zeit / self.dauer_aufklapp_vorgang

        # nun muss für jeden der 6 Motoren die s rampe berechnet werden, dafür wird eine for schleife genutzt
        for index in range(6):
            # jetzt wird die Hilfmethode für die S Rampe aufgerufen
            neuer_winkel = self.hilfsmethode_s_ramp(self.winkel_ruhelage_roboterarm[index], self.winkel_ziel_position_roboterarm[index], tau)

            # der Rückgabewert neuer Winkel muss nun in den Attributen aktualisiert werden
            self.aktuelle_winkel[index] = neuer_winkel

        # die Methode ist fertig wenn tau gleich 1 ist --> ab dann muss das Attribut self.aktueller modus geändert werden
        if (tau >= 1):

            # --> so kann ab dann die Methode Teleoperation die Steuerung übernehmen
            self.aktueller_modus = "TELEOPERATION"

        # abschließend müssen die aktualisierten aktuellen Winkel zurückgegeben werden
        return self.aktuelle_winkel.copy()

    def modus_einklappen(self):
        # diese Methode steuert das Einklappen des Roboterarms

        # zuerst muss die bis zum jetzigen Zeitpunkt verstrichene Zeit augelesen werden
        verstrichene_zeit = time.time() - self.startzeit_einklappen
        
        # tau --> verstrichene Zeit durch die festgelegte Aufklappdauer
        tau = verstrichene_zeit / self.dauer_einklapp_vorgang
        
        # nun muss für jeden der 6 Motoren die s rampe berechnet werden, dafür wird eine for schleife genutzt
        # Ziel Winkel beim Einklappen sind die gleichen wie die winkel_ruhelage
        for index in range(6):
            # jetzt wird die Hilfmethode für die S Rampe aufgerufen
            neuer_winkel = self.hilfsmethode_s_ramp(self.winkel_vor_einklappen[index], self.winkel_ruhelage_roboterarm[index], tau)
        
            # der Rückgabewert neuer Winkel muss nun in den Attributen aktualisiert werden
            self.aktuelle_winkel[index] = neuer_winkel

        # die Methode ist fertig wenn tau gleich 1 ist --> ab dann muss das Attribut self.aktueller modus geändert werden
        if (tau >= 1):
        
            # --> Programm ist beendet
            self.aktueller_modus = "BEENDET"

        # abschließend müssen die aktualisierten aktuellen Winkel zurückgegeben werden
        return self.aktuelle_winkel.copy()
        


    
    def modus_teleoperation(self, geschwindigkeitswerte, dt):
        # diese Methode ist für die Steuerung zuständig wenn Geschwindigkeitswerte aus der Vision Klasse kommen
        # sie ist demnach der hilfmethode lineare Rampe übergestellt und steuert diese
        # Methode iteriert durch die Motoren und ruft für jeden die hilfmethode lineare Rampe auf



        # jetzt müssen die einzelnen Winkel für die Servo Motoren berechnet werden --> Nutzung einer for Schleife
        # enumerate entpackt gleichzeitig zwei Dinge: Position und Inhalt der Geschwindigkeitsliste welche von Vision Klasse bereitgestellt wird
        # Position der Zahl in der Liste --> index kriegt eine Ganzzahl
        # Inhalt der Liste --> der Inhalt an der Position der übergebenen Liste
        # Bsp: 1. Durchlauf: index = 0, geschwindigkeitswert = 10
        #      2. Durchlauf: index = 1, geschwindigkeitswert = 0
        #      3. Durchlauf: index = 2, geschwindigkeitswert = -5
        for index, geschwindigkeitswert in enumerate(geschwindigkeitswerte):

            # Das Zieltempo wird durch den jeweiligen Geschwindigkeitswert von der Vision Klasse mit der maximal Geschiwndigkeit für den jeweiligen Servo Motor multipliziert um das Ziel Tempo zu errechnen
            ziel_tempo = geschwindigkeitswert * self.max_geschwindigkeit[index]

            # nun wird Hilfsmethode lineare Rampe aufgerufen
            neuer_winkel, neue_geschwindigkeit = self.hilfsmethode_lineare_rampe(self.aktuelle_winkel[index], self.aktuelle_geschwindigkeiten[index], ziel_tempo, self.beschleunigungssrate[index], dt, self.max_winkel_limits[index], self.min_winkel_limits[index])

            # die zurückgegebenen Werte müssen nun in den Attributen in den jeweiligen Listen aktualisiert werden
            self.aktuelle_geschwindigkeiten[index] = neue_geschwindigkeit
            self.aktuelle_winkel[index] = neuer_winkel


        # wenn for Schleife 5 mal durchgelaufen ist kann string mit neuen Winkelinformationen für den Arduino sowie die Klasse für den digitalen Twin gepackt werden
        return self.aktuelle_winkel.copy()



        
    def hilfsmethode_lineare_rampe(self, aktueller_winkel, aktuelle_geschwindigkeit, ziel_tempo, beschleunigungsrate, dt, max_winkel_limit, min_winkel_limit ):

            # die Hilfsmethode erhält sämtliche Werte aus der ihr übergestellten Methode und berechnet dann lediglich für den einzelnen Servo Motor den neuen Winkel

            # zurückgegeben werden zwei Werte:
            # diese können dann durch die übergestellte Methode in den jeweiligen Listen aktualisiert werden
            # die neue geschwindigkeit
            # der neue Winkel


            # wenn jetzige Geschwindigkeit gleich mit Ziel geschwindigkeit ist muss einfach mit der aktuellen Geschwindigkeit weiter gerechnet werden
            # daher wird ist die neue Geschwindigkeit zuerst die aktuelle Geschwindigkeit und nachfolgend kann dann mit if geprüft werden ob die neue Geschwindigkeit angepasst werden muss
            neue_geschwindigkeit = aktuelle_geschwindigkeit
            
            # --> ist aktuelle Geschwindigkeit kleiner als Ziel Tempo
            # Fall A: Beschleunigen
            if aktuelle_geschwindigkeit < ziel_tempo:
                # nun kann Beschleunigungsrate mit dt multipliziert werden und auf die aktuelle Geschwindigkeit addiert werden
                neue_geschwindigkeit = aktuelle_geschwindigkeit + beschleunigungsrate * dt

                # neue Geschwindigkeit muss überprüft werden ob sie nicht über dem Ziel Tempo für den jeweiligen Motor liegt
                # Nutzung von min und max; Achtung min() --> Obergrenze, max() --> Untergrenze
                # falls Ziel Tempo kleiner als neue_geschwindigkeit wäre würde ziel_tempo als neue geschwindigkeit genutzt werden
                neue_geschwindigkeit= min(neue_geschwindigkeit, ziel_tempo)



            # ist aktuelle Geschwindigkeit größer als Ziel Tempo
            # Fall B: Bremsen
            elif aktuelle_geschwindigkeit > ziel_tempo:
              # nun kann Beschleunigungsrate mit dt multipliziert werden und von aktueller Geschwindigkeit abgezogen werden  
              neue_geschwindigkeit = aktuelle_geschwindigkeit - beschleunigungsrate * dt

              # neue Geschwindigkeit muss überprüft werden ob sie nicht über dem Ziel Tempo für den jeweiligen Motor liegt
              # Nutzung von min und max; Achtung min() --> Obergrenze, max() --> Untergrenze
              # falls Ziel Tempo größer als neue_geschwindigkeit wäre würde ziel_tempo als neue geschwindigkeit genutzt werden
              neue_geschwindigkeit = max(neue_geschwindigkeit, ziel_tempo)


            # Servomotoren können nur Winkel als Information aufnehmen --> Umrechnung von Geschwindigkeit in in Winkel
            # dafür wird jeweiliger Winkel des Motors plus die neue Geschwindigkeit * dt gerechnet
            neuer_winkel = aktueller_winkel + neue_geschwindigkeit * dt

            # nun muss überprüft werden ob dieser Winkel in den festgelegten Winkel Limits des Servo liegt --> Nutzung von min & max
            # Achtung min() --> Obergrenze, max() --> Untergrenze
            neuer_winkel = min(neuer_winkel, max_winkel_limit)
            neuer_winkel = max(neuer_winkel, min_winkel_limit)

            # nun müssen die Werte an die übergeordnete Methode zurückgegeben werden
            return neuer_winkel, neue_geschwindigkeit
            


    def hilfsmethode_s_ramp(self, startwinkel, zielwinkel, tau):
        # die hilfsmethode welche die S Rampe modelliert erhält lediglich den Startwinkel an dem Gelenk losfährt
        # den Zielwinkel wo das Gelenk am Ende stehen soll
        # den normierten Zeitfortschritt tau --> die Mathematik hinter der Berechnung rechnet nicht in Sekunden sondern in Prozent
        # --> wenn die Bewegung startet ist tau = 0.0 (0%)
        # --> nach Hälfte der Zeit ist tau = 0.5 (50%)
        # --> wenn Zeit abgelaufen ist, tau = 1.0 (100%)
        # dieses tau wird zuvor in der aktualisiere Winkel Schleife berechnet durch verstrichene Zeit / gesamt Zeit und wird dann übergeben (sofern die Hilfsmethode in einer anderen Methode genutzt wird, wird logischerweise dort das tau berechnet)

        
        # als Formel wird ein Polynom 5. Grades genutzt
        # s(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5;
        # die Koeffizienten sind Ergebnis eines LGS
        # siehe Herleitung in MATLAB

        # Randbedingungen: Position am Start: Bei tau = 0 --> s = 0 (startwinkel)
        #                  Position am Ziel: tau = 1 --> s = 1 (zielwinkel)
        #                  Geschwindigkeit am Start: v = 0
        #                  Geschwindigkeit am Ziel: v = 0
        #                  Beschleunigung am Start: a = 0
        #                  Beschleunigung am Ziel: a = 0
        
        # dadurch ergibt sich diese Gleichung mit den eingesetzten Koeffizienten
        # s(t) = 10*t^3 -15*t^4 + 6*t^5
      

        # 1. tau darf nur im Bereich zwischen 0.0 und 1.0 sein
        # um tau in diesem Bereich "einzusperren" Nutzung von min und max:

        # wenn tau größer als 1.0 ist wird 1.0 zurückgegeben
        tau = min(1.0, tau)

        # wenn tau kleiner als 0.0 Rückgabe von 0.0
        tau = max(0.0, tau)


        # 2. nun Nutzung des Polynom 5. Grades
        # Ergebnis ist ein Faktor zwischen 0.0 und 1.0
        s = 10 * (tau ** 3) - 15 * (tau ** 4) + 6 * (tau ** 5)


        # 3. Berechnung des tatsächlichen Winkels
        # Formel:
        # (zielwinkel - startwinkel) --> ist das delta welches der Arm bzw. das Gelenk zurücklegen muss
        # durch die multiplikation mit s kommt der Prozentwert ins Spiel. Bsp: s = 0.2, delta = 90° --> Ergebnis ist 18
        # diese 18 werden auf den Startwinkel addiert. Bsp: 45 + 18 = 63° --> diese 63° werden an den Servo übermittelt
        # ACHTUNG: das angenommene s ist gigantisch groß in der Realität liegt das tau am Anfang bei vlt 0.02 Sekunden und damit liegt bei s bei 0.000009 dadurch kommt am Anfang eine fast identische Gradzahl zu 45° heraus

        neuer_winkel = startwinkel + (zielwinkel - startwinkel) * s

        # 4. Rückgabe des neuen Winkel
        return neuer_winkel




        
    