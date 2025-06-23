{
  description = "Rust dev engiroment";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    fenix.url = "github:nix-community/fenix"; # Rust toolchain

  };
  outputs = { self, nixpkgs, flake-utils, fenix }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        rustToolchain = fenix.packages.${system}.stable.withComponents [
          "rustc"
          "cargo"
          "clippy"
          "rustfmt"
          "rust-src"
          "llvm-tools-preview"
        ];
        withTargets = with fenix.packages.${system};
          combine [
            rustToolchain
            targets.thumbv7m-none-eabi.stable.rust-std
            targets."thumbv8m.main-none-eabihf".stable.rust-std
          ];
      in {
        devShells.default = pkgs.mkShell {
          buildInputs = [
            withTargets
            pkgs.gcc-arm-embedded
            pkgs.pkg-config
            pkgs.libusb1
            pkgs.probe-rs
            pkgs.picotool
            # for displaying the plot
            pkgs.python314
            pkgs.gnuplot
          ];

        };
      });
}
