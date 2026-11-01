"""
=========================================================
PROJEKT:        Roboterarm mit prädiktiver Kollisionsvermeidung
DATEI:          Digital_Twin_Milestone_5.py
BESCHREIBUNG:   Schnittstelle zur PyBullet-Physik-Engine. Erzeugt 
                den sichtbaren Zwilling sowie einen unsichtbaren 
                Geister-Arm für die Zukunftssimulation und misst 
                dynamisch die Abstände zwischen Bauteilen und Hindernissen.
---------------------------------------------------------
ENTWICKLER:     Linus Schiel
VERSION:        v1.0.0 (Proof of Concept 1)
=========================================================
"""



import pybullet as p
import pybullet_data
import math

class Digital_Twin:

    # an Konstruktor wird zusätzlich noch der jeweilige urdf_pfad übergeben, so kann später in der main oder in einer anderen Klasse
    # einfach ein anderer Pfad übergeben wodurch ein ganz anderer Roboterarm (bspw. andere Hebelarme) gesteuert werden könnte
    def __init__(self, urdf_pfad):

        # PyBullet Verbindung starten & zusätzlich über p.GUI wird ein Graphical User Interface gestartet --> Rendern einer 3D Oberfläche
        # (GEgenstück ist p.Direct --> reines Rechnen im Hintergrund ohne grafische Oberfläche --> stärkere Perfomance 
        p.connect(p.GUI)


        # Entfernen der Sidebars in PyBullet
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)


        # Pybullet hat eigene Standard Dateien, Suchpfad setzen
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Laden des klassischen Kachelboden
        self.boden_id = p.loadURDF("plane.urdf")

        # Roboter laden
        # fileName: Absoluter Pfad der beim Aufruf der Klasse übergeben wird
        # basePosition: Verschiebt den Nullpunkt des Roboters [X,Y,Z]
        # useFixedBase = True: Bodenplatte wird am Boden "festgeschraubt", sonst würde Roboterarm umfallen, da in Py Bullet Umgebung reale Schwerkraft herrscht
        self.roboter_id = p.loadURDF(
            fileName = urdf_pfad,
            basePosition = [0,0,0],
            useFixedBase = True
        )

        # Laden des Roboter "Geists" --> dieser führt die Zukunftsimulationen durch und ist durchsichtig wodurch der eigtl Twin durch ständiges Reseten nicht zittert
        self.geist_id = p.loadURDF(
            fileName = urdf_pfad,
            basePosition =[0,0,0],
            useFixedBase = True
        )

        # Kollision zwischen Geist und Echt muss deaktiviert werden
        anzahl_gelenke = p.getNumJoints(self.roboter_id)
        for i in range(-1, anzahl_gelenke):
            # Geist leicht durchsichtig einfärben
            p.changeVisualShape(self.geist_id, i, rgbaColor=[0.5, 0.8, 1, 0.4])


            for j in range(-1, anzahl_gelenke):
                # Niemals Kollisionen zwischen Echt (j) und Geist (i) berechnen
                p.setCollisionFilterPair(self.geist_id, self.roboter_id, i, j, enableCollision=0)

    
        


    def hindernisse_erzeugen(self, groesse, position):
        # =========================================
        # Hindernisse dynamisch erzeugen
        # =========================================
        # erschaffen einer simpelen Box in PyBullet

        # Achtung: PyBullet rechnet nicht mit Gesamtlänge, Gesamtbreite, Gesamthöhe sondern mit einem Ankerpunkt welcher im Schwerpunkt sitzt und von dort aus geht es die Hälfte der Gesamtlänge in die jeweiligen Richtungen
        # --> REchnen in Half-Extents --> halbe LÄnge vom Mittelpunkt
        halbe_groesse = [groesse[0]/2, groesse[1]/2, groesse[2]/2]

        # unsichtbare Kraftfeld für Kollisionserkennung
        kollisions_form_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=halbe_groesse)

        # das sichtbare Modell für das GUI
        visuelle_form_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=halbe_groesse, rgbaColor=[1, 0, 0, 1]) # Rot

        # Objekt in der Umgebung installieren
        hinderniss_id = p.createMultiBody(                  # CreateMultiBody führt dazu dass Hitbox und Visuelles Modell miteinander kombiniert werden
            baseMass=0,                                     # Masse 0 --> das Objekt ist statisch und unbeweglich wie eine Wand
            baseCollisionShapeIndex=kollisions_form_id,     # Hitbox --> um Rechenzeit wird simple geometrische Box über Objekt gelegt
            baseVisualShapeIndex=visuelle_form_id,          # Visuelles Modell wird extrem detailliert angezeigt, Physik Engine ignoriert dieses Modell komplett
            basePosition=position                           # als Position für Objekt wird übergebene Liste verwendet
        )

        # Physischen Abprall Widerstand deaktivieren
        anzahl_gelenke = p.getNumJoints(self.roboter_id)
        for i in range(-1, anzahl_gelenke):
            # physische Kollision zwischen Echtem Arm und Hinderniss deaktivieren
            p.setCollisionFilterPair(self.roboter_id, hinderniss_id, i, -1, enableCollision=0)


        return hinderniss_id



    def setze_arm_auf_neue_winkel(self, winkel_liste):
        # Hauptmethode für die Zukunftsimulation
        # Anstatt Motor mit einer Kraft auf einen bestimmten Winkel zu fahren (kostet Zeit) "telepotiert"  resetJointState das Gelenk augenblicklich in die Zielpose
        # dadurch kann Simulation in millisekunden berechnet werden

        # im jetzigen POC gibt es das Greifer Gelenk nicht daher darf die Methode dieses Gelenk nicht simulieren
        # --> Abfrage wie viele Gelenke es jetzt gerade gibt
        anzahl_gelenke = p.getNumJoints(self.roboter_id)

        for index, winkel_grad in enumerate(winkel_liste):
            # enumerate gibt index (welcher Motor) und den passenden Winkel zurück

            # Schutzmechanismus: Überspringe alle Winkel, für die es kein reales Gelenk gibt
            if index >= anzahl_gelenke:
                continue

            winkel_radiant = math.radians(winkel_grad)

            # Gelenk direkt auf den Zielwinkel telepotieren
            p.resetJointState(
                bodyUniqueId = self.geist_id,        # welches Objekt, welcher Roboterarm soll bewegt werden wird durch zugehörige ID geklärt
                jointIndex = index,                  # Motor welcher angesteuert werden soll wird über Index gelöst
                targetValue = winkel_radiant         # Zielwinkel der "angefahren" werden soll
            )


    def twin_erlange_abstaende(self):
        # Methode scannt die gesamte Umgebung des Roboterarms und liefert ein Dictionary mit einem Tuple in welchem die HInderniss ID und Bauteil ID ist und dem kleinsten Abstand dieser Kombination

        # bevor die jeweiligen Abstände berechnet werden wird ein leeres Dictionary erstellt
        # der Roboterarm ist mit seinen Bauteilen durchnummeriert
        # Durchnummerierung der Gelenke --> von 0 bis 5 (bei 6 Achsen Roboterarm)
        # Bauteil Nummer sind wie folgt vergeben:
        # ID -1 --> Basis --> hat kein Gelenk unter sich (Ursprung)

        # ID 0 --> Schulter-Drehung:
        #                          - Gelenk 0: Der Motor, der auf der Basis (-1) sitzt
        #                          - Bauteil 0: Die Gabel der Schulter, die sich dreht wenn Motor 0 läuft
        # ID 1 --> Schulter-Neigung:
        #                          - Gelenk 1: DEr Motor, der seitlich in der Schulter-Gabel (0) sitzt
        #                          - Bauteil 1: Der lange Oberarm
        # ID 2 --> Ellenbogen:
        #                          - Gelenk 2: Der Motor am Ende des Oberarms (1)
        #                          - Bauteil 2: Der Unterarm
        # ID 3 --> Handgelenk-Drehung:
        #                          - Gelenk 3: Motor welcher das Handegelenk dreht
        #                          - Bauteil 3
        # ID 4 --> Handgelenk-Neigung:
        #                          - Gelenk 4: Motor welcher die Greifer Aufnahme bewegt
        #                          - Bauteil 4
        # ID 5 --> Greifer:
        #                          - Gelenk 5: Motor welcher die Greifer Backen bewegt
        #                          - Bauteil 5
        
        # als Abstand wird 2.0 Meter angenommen --> was für jedes Bauteil extrem sicher ist und noch keine Verzögerung eingeleitet werden muss
        bauteil_abstaende = {-1: 2.0, 0: 2.0, 1: 2.0, 2: 2.0, 3: 2.0, 4: 2.0, 5: 2.0}


        alle_gesammelten_datensaetze = []

        # dynamsiche Ermittlung der Anzahl aller geladenen Objekte (Roboter, Boden,  beliebige Hindernisse)
        anzahl_objekte = p.getNumBodies()

        # Schleife welche durch jedes einzelne Objekt in der Welt iteriert
        for i in range(anzahl_objekte):
            # Erlangen der spezifischen ID des jeweiligen Hinderniss
            body_b_id = p.getBodyUniqueId(i)

            # Roboter darf sich nicht mit sich selbst oder Geist vergleichen
            if body_b_id not in [self.roboter_id, self.geist_id]:
                # Abstand zwischen Roboter und dem aktuellen Umgebungsobjekt messen
                # distance=2.0 --> maximaler Suchradius um Rechenleistung zu sparen
                # Liefert eine Liste mit 14-teiligen Datenpaketen für dieses spezifische Objekt
                gefundene_abstands_datensaetze = p.getClosestPoints(bodyA=self.geist_id, bodyB=body_b_id, distance=2.0)

                # gefundene Messpunkte an Sammelliste anhängen
                alle_gesammelten_datensaetze.extend(gefundene_abstands_datensaetze)


        for punkt in alle_gesammelten_datensaetze:
            # Funktion getClosestPoints() gibt massiven Datensatz zurück, hierbei sind 14 Werte in einer Liste hinterlegt
            # um die Bauteil ID, dem das Hinderniss zu nahe kommt zu bekommen wird 3. Eintrag benötigt
            bauteil_id = punkt[3]

            # um exakten Abstand zwischen HInderniss und Bauteil zu bekommen, wird 8. Eintrag genutzt
            distanz = punkt[8]

            hindernis_id = punkt[2]

            # Ausnahme für die Basis, da diese immer auf dem Boden steht
            if hindernis_id == self.boden_id and bauteil_id == -1:
                continue


            # Sicherheitsprüfung, falls ein Bauteil-Index gefunden wird, der nicht im Dictionary steht
            if bauteil_id in bauteil_abstaende:
                # wenn der neue Abstand kleiner ist als der im Dictionary --> überschreiben
                if distanz < bauteil_abstaende[bauteil_id]:
                    bauteil_abstaende[bauteil_id] = distanz

        # Rückgabe der Liste
        return bauteil_abstaende


    def erlange_abstaende(self):
        # Methode scannt die gesamte Umgebung des Roboterarms und liefert ein Dictionary mit einem Tuple in welchem die HInderniss ID und Bauteil ID ist und dem kleinsten Abstand dieser Kombination
        
        # bevor die jeweiligen Abstände berechnet werden wird ein leeres Dictionary erstellt
        # der Roboterarm ist mit seinen Bauteilen durchnummeriert
        # Durchnummerierung der Gelenke --> von 0 bis 5 (bei 6 Achsen Roboterarm)
        # Bauteil Nummer sind wie folgt vergeben:
        # ID -1 --> Basis --> hat kein Gelenk unter sich (Ursprung)
        
        # ID 0 --> Schulter-Drehung:
        #                          - Gelenk 0: Der Motor, der auf der Basis (-1) sitzt
        #                          - Bauteil 0: Die Gabel der Schulter, die sich dreht wenn Motor 0 läuft
        # ID 1 --> Schulter-Neigung:
        #                          - Gelenk 1: DEr Motor, der seitlich in der Schulter-Gabel (0) sitzt
        #                          - Bauteil 1: Der lange Oberarm
        # ID 2 --> Ellenbogen:
        #                          - Gelenk 2: Der Motor am Ende des Oberarms (1)
        #                          - Bauteil 2: Der Unterarm
        # ID 3 --> Handgelenk-Drehung:
        #                          - Gelenk 3: Motor welcher das Handegelenk dreht
        #                          - Bauteil 3
        # ID 4 --> Handgelenk-Neigung:
        #                          - Gelenk 4: Motor welcher die Greifer Aufnahme bewegt
        #                          - Bauteil 4
        # ID 5 --> Greifer:
        #                          - Gelenk 5: Motor welcher die Greifer Backen bewegt
        #                          - Bauteil 5
        
        # Dicitionary ist anfangs leer --> später werden Tuples (bauteil_id, hindernis_id) mit dem dazugehörigen kürzesten Abstand gespeichert
        # Hinweis: Dictionary ungleich zu Liste
        # auf Liste wird mit Elementnummer auf den Wert zugegriffen --> liste[0]
        # auf Dictionary wird mit Schlüssel zugegriffen (2,4)
        bauteil_hindernis_abstaende = {}

        # während nachfolgende for Schleife durch die verschiedenen IDs iteriert werden mehrere Werte für eine Hinderniss Bauteil Beziehung berechnet diese Datensätze werden der alle_gesammlten_datensaetze Liste angehängt und später ausgewertet
        alle_gesammelten_datensaetze = []


        # dynamische Ermittlung der Anzahl aller geladenen Objekte (Roboter, Boden,  beliebige Hindernisse)
        anzahl_objekte = p.getNumBodies()
        
        # Schleife welche durch jede  einzelne ID (einzelne Objekte) in der Welt iteriert
        for i in range(anzahl_objekte):

            # Erlangen der spezifischen ID des jeweiligen Hinderniss
            body_b_id = p.getBodyUniqueId(i)

            # Roboter darf sich nicht mit sich selbst oder Geist vergleichen
            if body_b_id not in [self.roboter_id, self.geist_id]:

                # Abstand zwischen Roboter und dem aktuellen Umgebungsobjekt messen
                # distance=2.0 --> maximaler Suchradius um Rechenleistung zu sparen
                # Liefert eine Liste mit 14-teiligen Datenpaketen für dieses spezifische Objekt
                gefundene_abstands_datensaetze = p.getClosestPoints(bodyA=self.geist_id, bodyB=body_b_id, distance=2.0)

                # gefundene Messpunkte an Sammelliste anhängen
                alle_gesammelten_datensaetze.extend(gefundene_abstands_datensaetze)


        # for Schleife iteriert durch die angehängten einzelnen Listen welche zuvor erstellt wurden
        for messpunkt in alle_gesammelten_datensaetze:
            # Funktion getClosestPoints() gibt massiven Datensatz zurück, hierbei sind 14 Werte in einer Liste hinterlegt

            # um die Bauteil ID, dem das Hinderniss zu nahe kommt zu bekommen, wird 3. Eintrag benötigt
            bauteil_id = messpunkt[3]

            # um Hindernis ID, zu erlangen wird 2. Eintrag benötigt
            hindernis_id = messpunkt[2]

            # um kürzesten Abstand zwischen HInderniss und Bauteil zu bekommen, wird 8. Eintrag genutzt
            # (Hinweis: in Liste können mehrere Abstände drin sein)
            distanz = messpunkt[8]


            # die Basis darf nicht mit dem Boden verglichen werden da diese direkt auf dem Boden steht
            if hindernis_id == self.boden_id and bauteil_id == -1:
                continue

            # Erstellen der Tuples
            # Beispiel Tuples:
            # {
            # (2, 5): 0.20,      (Unterarm, Tischboden) --> Abstand: 0.20m
            # (2, 4): 0.30,      (Unterarm, Kiste A)    --> Abstand: 0.30m
            # (3, 6): 0.80       (Handgelenk, Kiste B)  --> Abstand: 0.80m
            #}

            bauteil_und_hindernis = (bauteil_id, hindernis_id)


            # Eintrag ins Dictionary
            # als erstes Prüfung ob es bereits einen Eintrag (Tuple) mit genau dieser Kombi (bauteil_id, hindernis_id) gibt
            if bauteil_und_hindernis not in bauteil_hindernis_abstaende:
                # wenn kein Eintrag vorhanden ist, muss diese Kombi neu angelegt werden
                bauteil_hindernis_abstaende[bauteil_und_hindernis] = distanz

            # wenn Tuple bereits vorhanden ist muss geprüft werden welcher Abstand kleiner ist
            elif distanz < bauteil_hindernis_abstaende[bauteil_und_hindernis]:
                # wenn die neue Distanz des Tuples kleiner ist muss der Wert des Tuples überschrieben werden
                bauteil_hindernis_abstaende[bauteil_und_hindernis] = distanz


        # Rückgabe des Dictionary
        return bauteil_hindernis_abstaende




    def aktualisiere_twin(self, aktuelle_winkel_liste):
        # Schleife läuft über alle 6 Winkel, die von der Kinematik kommen

        # enumerate liefert den Index (i) und den Winkel in Grad gleichzeitig
        for index, winkel_grad in enumerate(aktuelle_winkel_liste):

            # ---Temporärer Einschub ---
            # da die URDF aktuell nur 1 Gelenk hat, wird der Schleifendurchlauf für alle Gelenk ab Index 4 abgebrochen, sonst stürzt PyBullet ab
            if index > 4:
                continue
            #-----------------

            # Pybullet benötigt Winkel im Bogenmaß --> Umrechnung
            winkel_radiant = math.radians(winkel_grad)

            # Motor (/Gelenk) ansteuern
            p.setJointMotorControl2(
                bodyIndex=self.roboter_id,      # jedes objekt hat eine ID, mit diesem Parameter (im Konstruktor definiert) ist für PyBullet klar es handelt sich um den Roboterarm
                jointIndex = index,             # Welches Gelenk des Roboters soll bewegt werden, URDF-Datei nummeriert Gelenke strikt von 0 bis 1 durch
                controlMode=p.POSITION_CONTROL, # Motoren können in der Robotik auf drei Arten geregelt werden:
                                            # Velocity Control --> Drehe dich unendlich mit 10 U/min weiter
                                            # Torque Control --> Drücke mit 5 Nm gegen den Widerstand, egal bei welchem Winkel du landest (Wichtig bspw. beim greifen)
                                            # Position Control --> Motor soll exakt auf diese Koordinate fahren und sie halten
                targetPosition=winkel_radiant,  # Zielwert auf den Motor fahren soll (in rad)
                force=5000,                       # maximales Drehmoment, die virtueller Motor aufwenden darf um Zielwinkel zu erreichen
            )



        # Simulation läuft nicht kontinuierlich, sondern in Frames wie ein Daumenkino
        # Aufruf von p.stepSimulation() berechnet einen exakten Zeitschritt in die Zukunft, in diesen Millisekunden berechnet die Engine die komplette Physik
        p.stepSimulation()


    def trennen(self):
        # trennen der Verbindung
        p.disconnect()

