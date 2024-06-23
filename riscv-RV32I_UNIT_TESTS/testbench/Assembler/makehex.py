def format_hex_data(hex_data, start_address):
    # Split the hex data into a list
    hex_list = hex_data.split()
    
    # Group the hex data into chunks of 16
    chunks = [hex_list[i:i+16] for i in range(0, len(hex_list), 16)]
    
    # Initialize the formatted output
    formatted_output = []
    
    # Loop through each chunk and format it
    for i, chunk in enumerate(chunks):
        # Create the address part of the line
        address = f"@0000{start_address + i:03x}"
        # Join the chunk in reverse order with underscores
        data = "_".join(chunk[::-1])
        # Combine the address and data
        formatted_line = f"{address} {data}"
        # Add the formatted line to the output list
        formatted_output.append(formatted_line)
    
    # Join the formatted output lines with newlines
    return "\n".join(formatted_output)

def read_file(filename):
    with open(filename, 'r') as file:
        return file.read().strip()

def write_file(filename, data):
    with open(filename, 'w') as file:
        file.write(data)

# Read hex data from data.txt
data_hex = read_file('data.txt')

# Read hex data from instructions.txt
instructions_hex = read_file('instructions.txt')

# Read comments from comments.txt
comments = read_file('comments.txt')

# Convert the hex data from data.txt (addresses 0->10)
formatted_data_hex = format_hex_data(data_hex, 0)

# Convert the hex data from instructions.txt (addresses 10->255)
formatted_instructions_hex = format_hex_data(instructions_hex, 0x10)

# Format comments
formatted_comments = "\n".join([f"// {line}" for line in comments.splitlines()])

# Combine both formatted outputs and comments
combined_output = f"{formatted_data_hex}\n{formatted_instructions_hex}\n{formatted_comments}"

# Write the combined formatted hex data and comments to data.hex
write_file('data.hex', combined_output)

print("Formatted hex data and comments written to data.hex")
