def convert_to_ascii(s):
    return [ord(char) for char in s]

def main():
    file_path = "test.txt"

    while True:
        command = input()
        with open(file_path, 'a+') as test_file:
            if command == "l":
                test_file.seek(0)
                lines = test_file.readlines()
                for i, line in enumerate(lines, start=1):
                    print(f"{i}. {line.strip()}")
                if not lines:
                    print("No any command.")
            elif command == "n":
                test_file.truncate(0)
                print("File cleared.")
            else:
                test_file.write(command + "\n")

if __name__ == "__main__":
    main()