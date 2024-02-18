import csv

def clean_csv(file_path, k):
    """
    Loops through a CSV file and creates a new file with only one out of every K lines.

    Args:
        file_path (str): Path to the original CSV file.
        k (int): The interval for keeping lines (e.g., k=3 means keep every 3rd line).
    """

    with open(file_path, 'r') as original_file, \
         open(f"{file_path}_cleaned.csv", 'w', newline='') as cleaned_file:
        reader = csv.reader(original_file)
        writer = csv.writer(cleaned_file)

        line_counter = 0
        for row in reader:
            line_counter += 1
            if line_counter % k == 0:
                writer.writerow(row)

# Example usage:
file_path = "points_map_in_the_lab_original.csv"  # Replace with the actual file path
k = 10  # Keep every kth line
clean_csv(file_path, k)
