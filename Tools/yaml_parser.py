import yaml, sys, pathlib, argparse

FW_VERSION_KEY = "FIRMWARE_VERSION"

def main(yaml_file:str, which_board:str, env_file:str, print_cgf:bool):
    yaml_file = pathlib.Path(yaml_file).resolve().as_posix()

    env_contents = []

    with open(yaml_file, "r") as stream:
        config = yaml.safe_load(stream)

        if print_cgf:
            boards = [key for key in config.keys() if key != FW_VERSION_KEY]
            print("\n".join(boards))
            exit(0)

        if which_board not in config.keys():
            print(f"Error:\tBoard config \"{which_board}\" is not in the \"{yaml_file}\" file.")
            exit(2)

        value = config.get(FW_VERSION_KEY)
        if value is None:
            print(f"Error:\tFIRMWARE_VERSION is missing from the \"{yaml_file}\" file.")
            exit(3)
        env_contents.append(f"{FW_VERSION_KEY}={value}")

        pairs = config.get(which_board)
        for key in pairs.keys():
            env_contents.append(f"{key.upper()}={pairs.get(key)}")

    if not print_cgf:
        env_file = pathlib.Path(env_file).resolve().as_posix()
        with open(env_file, "w") as file:
            for contents in env_contents:
                print(contents, file=file)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', "--file", metavar="YAML File", help="Boards configuration input YAML file.")
    parser.add_argument('-b', "--board", metavar="Board" ,help="Desired board for config generation.")
    parser.add_argument('-o', "--output_file", metavar="Output File", help="Specify the output generated .env file.")
    parser.add_argument("-p", "--print_configs", action="store_true", help="Print all the available board configurations and return.")
    
    args = parser.parse_args()
    yaml_file = args.file
    which_board = args.board
    env_file = args.output_file
    print_configs = args.print_configs

    if yaml_file is None:
        parser.print_help()

    if print_configs == False and (which_board is None or env_file is None):
        parser.print_help()
        exit(1)

    main(yaml_file, which_board, env_file, print_configs)
