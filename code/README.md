### How To
Mit ST link v2.0
1. `cargo build --release`
2. `probe-rs run --connect-under-reset --chip STM32f103C8 target/thumbv7m-none-eabi/release/water_controller` 
    - abhängig natürlich von projectname und chip zu dem compiled wird
3. Reset knopf drücken, kurz nachdem das hochladen gestartet hat.
    - kann mehrere Versuche brauchen