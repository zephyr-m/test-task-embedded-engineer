{
  description = "PMSM FOC current loop controller test task";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }:
    let
      supportedSystems = [ "x86_64-linux" "aarch64-linux" ];
      forAllSystems = nixpkgs.lib.genAttrs supportedSystems;
    in
    {
      devShells = forAllSystems (system:
        let
          pkgs = import nixpkgs { inherit system; };
        in
        {
          default = pkgs.mkShell {
            packages = with pkgs; [
              python3
              gcc
              gnumake
            ];

            shellHook = ''
              echo "Dev shell ready:"
              echo "  python3 simulation/current_loop_sim.py"
              echo "  gcc -std=c99 -Wall -Wextra -Werror -Istm32/Inc -c stm32/Src/current_loop_controller.c -o /tmp/current_loop_controller.o -lm"
            '';
          };
        });

      checks = forAllSystems (system:
        let
          pkgs = import nixpkgs { inherit system; };
        in
        {
          python-syntax = pkgs.runCommand "current-loop-python-syntax"
            {
              nativeBuildInputs = [ pkgs.python3 ];
            } ''
            cd ${self}
            python3 -m py_compile simulation/pi_controller.py simulation/current_loop_sim.py
            touch $out
          '';

          c-compile = pkgs.runCommand "current-loop-c-compile"
            {
              nativeBuildInputs = [ pkgs.gcc ];
            } ''
            cd ${self}
            gcc -std=c99 -Wall -Wextra -Werror -Istm32/Inc \
              -c stm32/Src/current_loop_controller.c \
              -o current_loop_controller.o
            touch $out
          '';
        });
    };
}

