import argparse
import re


def format_urdf(input_file_name, output_file_name=None):
    """
    Take an input file and formats the file content.

    Adds a new line before each <link name> and <joint name> element.

    :param input_file_name: str, name of the input file
    :param output_file_name: str, name of the output file. If None,
    modifies the input file in place.
    """
    with open(input_file_name) as file:
        content = file.read()

    # Add a new line before each <link name> and <joint name> element using regex substitution
    formatted_content = re.sub(r'(\s*<link name)', r'\n\1', content)
    formatted_content = re.sub(r'(\s*<joint name)', r'\n\1', formatted_content)

    # If output_file_name is None, modify the input file in place
    if output_file_name is None:
        output_file_name = input_file_name

    with open(output_file_name, 'w') as file:
        file.write(formatted_content)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process URDF files.')
    parser.add_argument('input_file', type=str, help='Input URDF file name')
    parser.add_argument('output_file', type=str, nargs='?', default=None,
                        help='Output URDF file name (optional)')

    args = parser.parse_args()

    format_urdf(args.input_file, args.output_file)
