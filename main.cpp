#include <iostream>
#include <iomanip>
#include "core/Entity.h"
#include "core/Radar.h"
#include "core/Tracker.h"

int main() {
    Entity aereo(0, 0, 10, 2); // Parte da 0,0 e va a 10m/s in X e 2m/s in Y
    Radar radar(3.0);          // Deviazione standard di 3 metri (abbastanza rumoroso)
    Tracker tracker;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Tempo | Reale X | Radar X | Stima X" << std::endl;
    std::cout << "------------------------------------" << std::endl;

    for (int t = 0; t < 20; t++) {
        aereo.update(1.0); // Muoviamo l'aereo di 1 secondo
        State reale = aereo.getState();

        auto [xR, yR] = radar.detect(reale.x, reale.y);

        tracker.ProcessData(xR, yR);
        State stima = tracker.GetEstimatedState();

        std::cout << t << "s    | " 
                  << reale.x << "  | " 
                  << xR << "  | " 
                  << stima.x << std::endl;
    }

    return 0;
}