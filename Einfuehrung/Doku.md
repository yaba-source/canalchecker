Abfrage beim CLient nach Toleranz und Ziel


wenn der Roboter Postion die Position an server schickt berechnet Server Abstand und Winkel zum Ziel

    wenn Winkel Fehler größer Toleranz ist 
        Dann soll der Roboter gedreht werden
        Server sendet Drehbefehl 

    Wenn nicht dann fahre den Roboter 
        Server sendet cmd/vel


wenn Ist Pose = Soll Pose 
dann Ziel erreicht 
schick Ziel erreicht 
mache Roboter aus 
Wenn nicht dann schicke Position 