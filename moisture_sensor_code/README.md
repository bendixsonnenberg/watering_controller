### How To
Mit ST link v2.0
1. `cargo build --release`
    - es ist wichtig mit release zu compilen, da die Temperatursensoren hohe Tastfrequenzen benötigen, was mit Debug nicht funktioniert
2. `probe-rs run --connect-under-reset --chip STM32f103C8 target/thumbv7m-none-eabi/release/water_controller` 
    - abhängig natürlich von projectname und chip zu dem compiled wird
3. Reset knopf drücken, kurz nachdem das hochladen gestartet hat.
    - kann mehrere Versuche brauchen
