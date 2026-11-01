"""
=========================================================
PROJEKT:        Roboterarm mit prädiktiver Kollisionsvermeidung
DATEI:          Kollisionsvermeidung_Milestone_5.py
BESCHREIBUNG:   Sicherheitsfilter mit Look-Ahead-Logik. Simuliert 
                zukünftige Posen im digitalen Zwilling, berechnet 
                dynamische Bremswege und drosselt die Motor-
                geschwindigkeiten vor Hindernissen präventiv.
---------------------------------------------------------
ENTWICKLER:     Linus Schiel
VERSION:        v1.0.0 (Proof of Concept 1)
=========================================================
"""


# Problemstellung:
# Bei der Kollisionsvermeidung muss der Roboterarm dauerhaft den Abstand gegenprüfen um sicherzustellen, dass dieser nicht in eine Bounding Box des Hindernisses fährt
# dabei soll sichergestellt werden dass wenn der Benutzer den Arm mit voller Geschwindigkeit (Hand ist komplett aus Totzone) in ein Hinderniss fahren würde der Roboter
# dynamisch die Geschwindigkeit herunterregelt bis der Roboterarm exakt mit dem vorgegeben Sicherheitsabstand vor dem Hinderniss stehen bleibt

# dabei ergeben sich mehrere Probleme zum einen darf der Roboterarm sobald er in die zweitkritische Zone (zu nahe am Hinderniss, aber noch ausreichend Weg um zu bremsen) nicht komplett gedrosselt werden,
# sondern es soll bspw. der Greifer oder die Drehung des Arms weiter mit voller Geschwindigkeit gesteuert werden können
# daher war der erste Lösungansatz den Roboterarm in Abhängigkeiten zu setzen, eine kinematische Kette zu bilden: Ein Roboterarm ist eine strikte Hierarchie, jedes Gelenk bewegt alle nachfolgenden Bauteile, aber niemals die vorherigen
# Achse 1 (Basis): bewegt den ganzen Arm
# Achse 3 (Ellbogen): Bewegt Unterarm, Handgelenk und Greifer
# Achse 6 (Greifer): Bewegt nur die Backen des Greifers
# Bsp: Die Position des Ellbogens (Achse 3) ist im Raum zu 100% unabhängig davon, was Achse 4,5 und 6 tun
# --> wenn also Ellbogen droht mit Kiste zu kollidieren, sind ausschließlich die Achsen 1,2 und 3 dafür verantwortlich, das Handgelenk und der Greifer können völlig frei und mit Maximalgeschwindigkeit weiterdrehen
# 
# und hier liegt ein großer Denkfehler vor: es lässt sich nicht sicherstellen bzw. ausschließen dass der Roboterarm kein Handgelenk besitzt, was sich parallel zum restlichen Arm bewegen kann und beim nach unten bewegen das Hinderniss berühren kann
# das Hinderniss würde in diesem Fall zwischen Basis und Greifer sein
# daher kann und darf die kinematische Kette nicht genutzt werden


# (vermeintliche) Lösung: Simulation einer Zukunft des Roboterarms (look ahead Logik):
# hierbei wird für jedes Gelenk ein neuer Zielwinkel berechnet die Engine PyBullet simuliert dann diese nahe Zukunft
# nun Prüfung welches Gelenk hat den kleinsten Abstand zum Hinderniss --> Drosselung (wie Drosselung funktioniert zweitrangig) dieses Gelenks
# --> Achtung Denkfehler: Beispiel: Motor 3 (Ellbogen) steht gerade völlig still, Motor 1 (Basis) dreht sich und rammt unbeteiligten Ellbogen in die Kiste
# --> hier greift wieder Prinzip der kinematischen Kette: Gelenk 1 (Motor 1) bewegt alle nachfolgenden Bauteile

# Lösung: Herausfinden welche Bewegungsrichtung den Abstand wirklich verringert
# --> Simulation von 6 seperaten Zukünften, dabei wird jeder Motor und dann noch jedes Bauteil einzeln betrachtet umso die jeweiligen Geschwindigkeitswerte dynamisch anzupassen
# Um die passende Verzögerung zu berechnen wird Abstand vom Hindernis zum Roboterbauteil in 3 Zonen aufgeteilt
# Grüne Zone: Roboterarm hat noch ausreichend Abstand zum Hinderniss kein Bremsen notwendig
# Gelbe Zone: diese Zone wird dynamisch berechnet in Abhängigkeit der gerade herrschenden Geschwindigkeit des Motors und der maximalen Verzoegerung
# --> in dieser Zone wird der Ziel Geschwindigkeitswert welcher durch die Klasse Vision (Händesteuerung) bereitsgestellt wird in Abhängigkeit des noch verbleibenden Bremsweg durchweg angepasst
# dadurch erreicht der Roboterarm seinen Stillstand genau vor einem definierten Sicherheitsabstand vom Hinderniss entfernt (hierbei gibt es allerdings Einschränkungen --> siehe nachfolgender Code)
# Rote Zone: diese Zone stellt den zuvor festgelegten Sicherheitsabstand dar, und der Roboterarm soll genau auf der Grenze stoppen

# Hierbei wurde bei näherem Testen herausgefunden das ein gewisses Zittern durch die Simulation des "echten" digitalTwin stattfindet daher wird ein Geister Twin erzeugt der unsichtbar ist und dadurch nie zurückgesetzt werden muss
# (erzeugen dieses Geists findet in DigitalTwin Klasse statt)



class Kollisionsvermeidung:
    def __init__(self, digital_twin, max_verzoegerung,look_ahead_time, sicherheitsabstand=0.02, warning_time=0.5):

        
        self.twin = digital_twin                        # --> Referenz auf DigitalTwin Klasse, Objekt um Pybullet zu nutzen wird übergeben
        self.max_verzoegerung = max_verzoegerung        # --> maximale Verzögerung der Rampe (abhängig der Hardware, nachträglich sollen solche Werte in eine txt.datei/ urdf datei eingetragen werden und übergeordnet ausgelesen werden und an die einzelnen Klassen übergeben werden)
        self.look_ahead_time = look_ahead_time          # --> wie viele Sekunden in Zukunft simuliert wird
        self.sicherheitsabstand = sicherheitsabstand    # --> Sicherheitsabstand der zu den Hindernissen eingehalten werden soll in Metern (jetzt festgelegt 0.02m = 2cm, kann später überschrieben werden)
        

    def sicherheitsfilter_anwenden(self, ziel_geschwindigkeiten, jetzige_geschwindigkeiten, jetzige_winkel):

        # Liste in der später die sicheren Geschwindigkeiten eingetragen werden
        sichere_geschwindigkeiten = []
        
        # Visualisierung Dashboard --> Werte initialisieren ( Werte sind Platzhalter)
        kritischster_abstand_gesamt = 2.0
        kritische_gelbe_zone_gesamt = 0.0
        kleinster_faktor_gesamt = 1.0


        # ==========================================
        # SCHRITT 1: GEGENWART MESSEN 
        # ==========================================
        # "Geist" Twin zuerst auf exakte Position des echten Roboters teleportieren
        self.twin.setze_arm_auf_neue_winkel(jetzige_winkel)

        # Gegenwart messen --> Rückgabe eines intelligenten Tupel: (bauteil_id, hindernis_id)
        status_0_abstaende = self.twin.erlange_abstaende()



        # ==========================================
        # SCHRITT 2: JEDES GELENK ISOLIERT TESTEN
        # ==========================================
                
        # Iterieren durch for Schleife um jedes einzelne Bauteile getrennt zu betrachten
        for i in range(6):  # Achtung: Hardcoded --> später muss Wert dynamisch aus einer übergeordneten Struktur erlangt werden
            # Zukunft nur für das eine Gelenk des einen Bauteils simulieren
        
            # Die zukünftigen Winkel berechnen

            # einfache Formel mit s = v * t --> hierbei kann die Beschleunigung, weitere Parameter wie die eigtl exakte Geschwindigkeit zu diesem Zeitpunkt egal sein,
            # weil lediglich Interesse an der Tendenz der einzelnen Gelenke herrscht --> bewegt sich das Gelenk in der Zukunft in die Richtung des Hindernisses oder in die andere Richtung
         
            # zuerst werden die zukuenftigen Winkel für den Roboterarm berechnet, hierbei wird bei jedem Durchlauf nur EIN Winkel verändert um so herauszufinden welcher Motor dran Schuld ist dass der Roboter dem HInderniss zu nahe kommt
            # die restlichen Winkel bleiben gleich, Bsp: Durchgang 5 --> Nur Veränderung des Winkel 5 die restlichen bleiben gleich
            # um das zu erreichen wird bei jeder Iteration eine neue Kopie der Liste jetzige Winkel erstellt, dadurch wird immer nur ein Winkel verändert und die anderen bleiben
            zukuenftige_Winkel_fuer_dieses_Gelenk = jetzige_winkel.copy()


            # da ziel_geschwindigkeiten nur die reinen Sensorwerte der Vision Klasse sind, in der Kinematik Klasse aber mit einem Beschleunigungsfaktor gearbeitet wird
            # muss das in der Kollisionsvermeidung exakt gleich abgebildet sein
            # Servo Motoren können andere Geschiwndigkeitsfaktoren haben --> Prüfung

            # für Greifer (index 5) ist Maximalgeschwindigkeit nur auf 1.0 festgelegt
            if i == 5:
                max_kinematik_speed = 1.0

            # für alle anderen Haupt Achsen (0 bis 4) liegt die Maximalgeschwindigkeit bei 9.0
            else:
                max_kinematik_speed = 9.0

            # HINWEIS:
            # Werte wie index 5 für Greifer oder auch die Geschwindigkeitsfaktoren sind hard gecoded das muss später durch eine txt Datei oder andere Formate verändert werden
            # Für den jetzigen PoC1 ist das nicht mehr vorgesehen
            # Zusätzlich muss beachtet werden, dass der Greifarm bisher nicht berücksichtigt wird, hierfür wird im nächsten PoC2 eine eigene Betrachtung etabliert, da sich der Greifarm ganz anders verhält wie die restlichen Gelenke


            echte_grad_pro_sekunde = ziel_geschwindigkeiten[i] * max_kinematik_speed

            
            # berechnen der Winkeländerung --> wie viel muss auf den jetzigen Winkel addiert werden um den absoluten neuen Winkel zu erhalten
            delta_winkel = echte_grad_pro_sekunde * self.look_ahead_time
        
            # in kopierte Liste wird der neue veränderte Winkel eingetragen
            zukuenftige_Winkel_fuer_dieses_Gelenk[i] = jetzige_winkel[i] + delta_winkel
        
            # Aufrufen der Methode um den neuen Winkel zu simulieren
            self.twin.setze_arm_auf_neue_winkel(zukuenftige_Winkel_fuer_dieses_Gelenk)
        
            # Erlangen der Zukunftswerte, Abstände für ALLE Bauteile (Aufrufen der passenden Methode des digital twin)
            zukunft_abstaende = self.twin.erlange_abstaende()

            # ==========================================
            # SCHRITT 3: ALLE BAUTEILE ÜBERPRÜFEN
            # ==========================================
            # Annahme --> Optimalfal kein HIndernis --> Voller Schub möglich
            kleinster_geschwindigkeitsfaktor = 1.0

            for bauteil_und_hindernis in status_0_abstaende:
                # bauteil_und_hinderniss sind ein Tuple welches aus zwei Ids besteht z.B. (2,4)
                # --> 2 ist Unterarm, 4 ist rote Kiste (Hindernis)
                # Schleife prüft jede einzelne Bauteil Hindernis Beziehung

                # zuerst Sicherheitsprüfung da PyBullet aus Perfomance gründen nur im Radius von 2 Meter Hindernisse sucht
                # könnte sich Roboterarm weg vom Hinderniss bewegen und die 2 Meter überschreiten dadurch wäre das Tuple nicht mehr in den zukunft_abstaende und eine später Berechnung würde einen Fehler schmeißen
                if bauteil_und_hindernis not in zukunft_abstaende:
                    continue


                # für das Vergleichen der Distanzen der Bauteile werden die Distanzwerte des Ausgangsstatus (status_0) und die der Zukunft gebraucht
                # die Abstände/Distanzen sind immer vom jeweiligen Bauteil des Roboterarms (Bsp: Schulter) zu den Hindernissen gemessen
                status_0_abstand = status_0_abstaende[bauteil_und_hindernis]
                zukunft_abstand = zukunft_abstaende[bauteil_und_hindernis]


                
                # Fall 1: Dieses spezifische Bauteil bewegt sich WEG vom Hinderniss oder bleibt gleich
                
                # hierbei wird nicht nur generell ein Abstand verglichen
                # Problem hierbei wäre: der Roboterarm würde immer den kritischsten Abstand berechnen
                # Beispiel: Der Roboterarm hat zwei Hindernisse welche kritisch werden könnten, Decke mit 1m Abstand und Kiste  mit 5m Abstand
                # der Roboterarm würde den kritischsten Abstand berechnen --> 1 Meter Decke
                # jetzt bewegt sich der Roboterarm Geist voraus und er steuert zwei Meter auf die Wand zu
                # Zukunft messen: kleinster Abstand --> weiterhin 1 m Decke
                # Vergleich: 1m (Gegenwart) - 1m (Zukunft) = 0m gefahren
                # --> Vollgas
                # die Wand die allerdings von 5 auf 3 Meter herangerückt ist wurde komplett ignoriert

                # Lösung: Das vergleichen des Tuples mit (Bauteil, Hindernis) zwingt Code alle Hindernisse zu prüfen
                # Beispiel Tuples:
                # {
                # (2, 5): 0.20,      (Unterarm, Tischboden) --> Abstand: 0.20m
                # (2, 4): 0.30,      (Unterarm, Kiste A)    --> Abstand: 0.30m
                # (3, 6): 0.80       (Handgelenk, Kiste B)  --> Abstand: 0.80m
                #}
                
                # --> es wird immer die Gegenwart der Abstand des jeweiligen Hinderniss (Kiste A) mit dem Bauteil (Unterarm) mit dem Zukunftsabstand genau dieser Kombi HInderniss A und Unterarm verglichen
                

                if zukunft_abstand >= status_0_abstand:
                    continue # --> schaue nächstes Bauteil an



                # Fall 2: Bauteil rückt näher an Bauteil heran
                # hierbei muss streng auf das sanfte Abbremsen des Servos geachtet werden, um Materialversagen komplett zu vermeiden und Schwingungen auf ein minimum zu reduzieren
                # daher werden dynamisch zwei Zonen berechnet
                
                # gelbe Zone: stellt den dynamischen Bremsweg da, die größe des Bereichs (Bremsweg) hängt von der Geschwindigkeit des einzelnen Motors ab
                # --> fährt Motor mit Vollgas ist gelbe Zone extrem groß, ist Motor extrem langsam, ist Zone wenige mm
                
                # Funktionsweise: Roboter überquert äußere Linie (Geschwindigkeitsfaktor = 1.0) --> je tiefer der Roboter in die Zone fährt umso weiter drückt der Code den Faktor in Richtung 0
                # weil gelbe Zone exakt so breit ist wie Bremsweg des Motors stoppt der Motor exakt auf der Grenze der roten Zone (im theoretisch besten Fall)
                # Vorsicht: Der Roboterarm wird nie exakt auf dieser Grenze stoppen weil es mehrere Störgrößen und bottle necks gibt --> Bspw. Berechnungszeit des Codes, Latenz im Kabel zum Arduino, Schleppfehler des Servos
                                
                # Rote Zone: In diesem Bereich ist die Geschwindigkeit auf 0.0 gesetzt, hierbei wird der festgelegte Sicherheitsabstand vom Hinderniss als breite der Zone genutzt
                # Vorsicht: wenn ein Bauteil plötzlich in diese Grenze gesetzt wird, hat der Roboterarm keine Chance mehr passend abzubremsen --> der Roboter wird also in das Hinderniss chrashen
                # Gründe sind hierfür die baulichen Gegebenheiten weil ein Servo nur mit einer bestimmten Gegenbeschleunigung abbremsen kann



                # Berechnung der Breite der gelben Zone
                # zuerst wird der Betrag der Ziel Geschwindigkeit des betrachteten Motor (/Gelenk) benötigt
                motor_ziel_geschwindigkeit = abs(ziel_geschwindigkeiten[i])


                # Wenn Vision Klasse 0 liefert (Hand steht oder ist in Totzone), kann Berechnung des Bremswegs übersprungen werden
                # Hinweis: Bei Media Pipe oder anderen Sensoren meist rauschen der Werte wodurch 0.000056 herauskommen könnte obwohl Hand in Totzone ist, dieses Problem wird allerdings komplett in der vision KLasse behoben wodurch nur integer von -10 bis 10 als Geschwindigkeitswerte übergeben werden
                # Allerdings ergibt sich dadurch ein weiteres Problem bei ganz langsamen Handbewegungen bei denen ein Wert unter 1 berechnet wird wird durch int() radikal auf 0 abgeschnitten
                # Problem: Im Terminal bleiben die 100 % Schub als letzte Berechnung gespeichert, wenn jetzt allerdings eine 0 von Vision kommt muss die Berechnung übersprungen werden da sonst ein Division by zero error geschmissen werden würde
                # der 100 % Wert wird also gar nicht mehr angepasst und bleibt bei 100% obwohl der Roboterarm schon gar kein Schub mehr hat, Geschwindigkeits Wert ist ja auf 0
                # --> Lösung: Daher wird durch passende Prüfung sichergestellt das die Werte angepasst werden, sodass 0% Schub im Terminal stehen

                if motor_ziel_geschwindigkeit == 0:

                    # bevor Berechnung für iterierten Motor abgebrochen werden darf, muss geprüft werden ob Bauteil bereits in der roten Zone ist
                    if status_0_abstand <= self.sicherheitsabstand:

                        # falls Arm in roter Zone steht, darf Faktor nicht auf 100 % Prozent stehen bleiben
                        # wenn Faktor noch größer als 0.0 ist --> Sicherheitsmaßnahme: --> Zwingen des Codes auf 0.0
                        if 0.0 < kleinster_geschwindigkeitsfaktor:
                            kleinster_geschwindigkeitsfaktor = 0.0

                            # damit auch visuelle Anzeige den wahren Abstand ausgibt --> Überschreiben
                            kritischster_abstand_gesamt = status_0_abstand

                        # der globale Faktor, der im Terminal ausgegeben wird muss ebenfalls knallhart auf 0.0 gesetzt werden
                        if 0.0 < kleinster_faktor_gesamt:
                            kleinster_faktor_gesamt = 0.0


                    # Nachdem Sicherheitscheck für Geschwindigkeit 0 abgeschlossen ist, kann und muss die Berechnung übersprungen werden
                    continue



                # da ziel_geschwindigkeiten nur die reinen Sensorwerte der Vision Klasse sind, in der Kinematik Klasse aber mit einem Beschleunigungsfaktor gearbeitet wird
                # muss das in der Kollisionsvermeidung exakt gleich abgebildet sein
                # Servo Motoren können andere Geschiwndigkeitsfaktoren haben --> Prüfung
                
                # für Greifer (index 5) ist Maximalgeschwindigkeit nur auf 1.0 festgelegt
                if i == 5:
                    max_kinematik_speed = 1.0
                
                # für alle anderen Haupt Achsen (0 bis 4) liegt die Maximalgeschwindigkeit bei 9.0
                else:
                    max_kinematik_speed = 9.0
                
                # HINWEIS:
                # Werte wie index 5 für Greifer oder auch die Geschwindigkeitsfaktoren sind hard gecoded das muss später durch eine txt Datei oder andere Formate verändert werden
                # Für den jetzigen PoC1 ist das nicht mehr vorgesehen
                # Zusätzlich muss beachtet dass der Greifarm bisher nicht berücksichtigt wird, hierfür wird im nächsten PoC2 eine eigene Betrachtung etabliert, da sich der Greifarm ganz anders verhält wie die restlichen Gelenke


                echte_geschwindigkeit = motor_ziel_geschwindigkeit * max_kinematik_speed

                # Berechnung des Bremsweg (Herleitung siehe Ipad Dokument)
                bremsweg_winkel = (echte_geschwindigkeit **2) / (2.0 * self.max_verzoegerung)

                # durch Zukunftssimulation kann berechnet werden wie viel Meter ist das jeweilige Bauteil dem Hindernis näher gekommen
                gefahrene_meter = status_0_abstand - zukunft_abstand

                #if gefahrene_meter < 0.001:
                    #gefahrene_meter = 0.001

                # Wie viele Grad hat sich der Motor in der simulierten Zukunft (look_ahead_time) bewegt?
                gefahrene_grad = echte_geschwindigkeit * self.look_ahead_time

                # Berechnen des dynamsichen Übersetzungsverhältnis:
                # --> wie viele Meter bewegt sich dieses spezifische Bauteil, wenn sich der Motor um 1 Grad dreht?
                meter_pro_grad = gefahrene_meter / gefahrene_grad

                # jetzt kann Bremsweg in Meter umgerechnet werden
                bremsweg_meter = bremsweg_winkel * meter_pro_grad


                # Damit der Roboterarm bzw. der Motor genau auf der Grenze mit Sicherheitsabstand zum Hinderniss stehen bleibt muss die gelbe Zone Bremsweg + Sicherheitabstand sein
                gelbe_zone = self.sicherheitsabstand + bremsweg_meter

                # Geschiwndigkeitsfaktor Berechnung wird erster auf Optimalfall gesetzt --> kein Hinderniss im Weg
                geschwindigkeitsfaktor = 1.0

                # wenn der Simulierte Abstand (zukunft abstand) kleiner gleich dem Sicherheitsabstand --> Rote Zone
                if status_0_abstand <= self.sicherheitsabstand:
                    geschwindigkeitsfaktor = 0.0 # Rot Zone --> Sollte bei normaler Fahrt nicht passieren


                # wenn der Simulierte Abstand kleiner als die gelbe Zone ist wird der Geschwindigkeitsfaktor angepasst
                elif status_0_abstand < gelbe_zone:
                    
                    # zuerst wird der verbleibende Bremsweg berechnet
                    verbleibender_bremsweg = status_0_abstand - self.sicherheitsabstand

                    # Skalierung des Geschwindigkeitsfaktors
                    geschwindigkeitsfaktor = verbleibender_bremsweg / bremsweg_meter



                # da in der for Schleife durch alle Tuples (Bauteil, Hinderniss) durchiteriert wird, kann es passieren dass pro Bauteil verschiedene Geschwindigkeitswerte für den Motor errechnet werden,
                # allerdings darf dann nur der kleinste Werte verwendet werden, daher prüft if der gerade errechnete Faktor kleiner ist als der bereits eingetragene Wert
                # Bsp: Greifer verlangt 0.2 und Basis 0.8 --> kleinster_Geschwindigkeitsfaktor = 0.2
                if geschwindigkeitsfaktor < kleinster_geschwindigkeitsfaktor:

                    # wenn jetziger Wert kleiner ist --> Überschreiben des alten Werts
                    kleinster_geschwindigkeitsfaktor = geschwindigkeitsfaktor


                    # jetzt prüfen der globalen System-Ebene (für das Terminal):
                    # ist dieser Faktor nicht nur für diesen Motor der kleinste, sondern der kleinste im gesamten System (über alle bisherigen geprüften Motoren und Bauteil hinweg)?
                    if geschwindigkeitsfaktor < kleinster_faktor_gesamt:

                        # wenn ja --> dieser Wert ist ab jetzt bottel neck des Roboters
                        # und er bestimmt die Schub Anzeige im Terminal
                        # Im Terminal werden immer nur die Werte des absoluten bottle necks angezeigt
                        kleinster_faktor_gesamt = geschwindigkeitsfaktor

                        # die restlichen Werte des Dashboard müssen zu diesem Geschwindigkeitswert gehören
                        kritischster_abstand_gesamt = status_0_abstand
                        kritische_gelbe_zone_gesamt = gelbe_zone



            # ==========================================
            # SCHRITT 4: GESCHWINDIGKEIT DROSSELN
            # ==========================================
            # Nachdem alle Bauteile für dieses Gelenk (Motor) betrachtet wurden, wird der strengste Faktor angewendet
            gedrosselte_geschwindigkeit = ziel_geschwindigkeiten[i] * kleinster_geschwindigkeitsfaktor

            # anhängen der neuen Geschwindigkeit an die zuvor erstelle Liste
            sichere_geschwindigkeiten.append(gedrosselte_geschwindigkeit)


        # ==========================================
        # SCHRITT 5: Ausgabe auf Terminal
        # ==========================================

        # Bewegungserkennung: Prüfen, ob von Vision Klasse ein aktiver Steuerbefehl zurückgegeben wird
        # Standardmäßig findet keine Steuerbewegung statt --> False
        hand_bewegt_sich = False

        # Iterieren durch sämtliche Rohdaten Befehle der Vision Klasse
        for geschwindigkeit in ziel_geschwindigkeiten:
            if geschwindigkeit != 0:
                hand_bewegt_sich = True
                break # --> Exit: Sobald ein Motor dreht, steht fest das Roboterarm in Bewegung ist



        # Dashboard im Terminal soll sich nur aktualisieren wenn aktive Steuerung herrscht
        if hand_bewegt_sich:

            # globaler kleinster Faktor wird für die Anzeige genommen
            anzeige_faktor = kleinster_faktor_gesamt


            # Übersetzen des mathematischen Faktors in lesbaren System Status
            if anzeige_faktor == 0.0:
                status = "STILLSTAND"
            elif anzeige_faktor < 1.0:
                status = "BREMSEN "
            else:
                status = "SICHER  "


            # kritische gelbe zone gesamt besteht mathematisch aus dem Bremsweg Plus dem Sicherheitsabstand
            # --> Nutzer ist aber nur der reine dynamische Bremsweg interessant
            # daher abzug des Sicherheitsabstands
            berechneter_bremsweg = kritische_gelbe_zone_gesamt - self.sicherheitsabstand

            # da PyBullet mit Gleitkommazahlen (Floats) rechnet, kann es zu winzigen Rundungsfehlern kommen
            # wenn Bremsweg theoreitsch 0 ist, könnte Computer -0.00001 errechnen
            # um unlogische negative Bremswege zu vermeiden deckeln des Werts auf 0
            if berechneter_bremsweg < 0:
                berechneter_bremsweg = 0.0

            # String zusammensetzen
            # \r --> Cursor wird an Zeilenanfang gesetzt um nicht ins unendliche zu scrollen
            dashboard = (
                f"\r[KOLLISIONSVERMEIDUNG] "
                f"Min. Abstand: {kritischster_abstand_gesamt:.3f}m | "
                f"Dyn. Bremsweg: {berechneter_bremsweg:.4f}m | "
                f"Status: {status} | "
                f"Schub: {int(anzeige_faktor * 100):3d}%    "
                )
            
                # end="" --> Automatischen Zeilenumbruch nach Print verhindern
                # flush=True -->Terminal wird gezwungen, Text auf Millisekunde genau auszugeben
            print(dashboard, end="", flush=True)

            
        # Rückgabe der sicheren Geschwindigkeiten
        return sichere_geschwindigkeiten
          

