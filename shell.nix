{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  packages = with pkgs; [
    python3
    gcc
    gnumake
  ];

  shellHook = ''
    echo "Dev shell ready:"
    echo "  make test"
    echo "  python3 simulation/current_loop_sim.py"
  '';
}

