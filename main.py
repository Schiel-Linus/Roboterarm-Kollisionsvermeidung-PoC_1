"""
=========================================================
PROJEKT:        Roboterarm mit prädiktiver Kollisionsvermeidung
DATEI:          main.py
BESCHREIBUNG:   Einstiegspunkt des Programms. Konfiguriert die 
                Startparameter (URDF-Pfad, Bypass-Schalter), 
                initialisiert die Manager-Klasse und startet 
                den zentralen Systemzyklus.
---------------------------------------------------------
ENTWICKLER:     Linus Schiel
VERSION:        v1.0.0 (Proof of Concept 1)
=========================================================
"""



from Manager import Manager


if __name__ == "__main__":

    roboter = Manager(
        urdf_pfad=r"cad_modell/urdf/poc_1_only_cad_step_03.urdf",
        kollisionsvermeidung_aktiv=True        # Möglichkeit zur Aktivierung des Bypasses um für Test-Zwecke die Kollisionsvermeidung zu deaktivieren
    )

    roboter.systemzyklus_ausfuehren()