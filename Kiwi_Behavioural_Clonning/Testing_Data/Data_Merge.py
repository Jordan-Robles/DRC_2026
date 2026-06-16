from pathlib import Path
import csv
import shutil

SOURCE_DIRS = [
    Path(r"C:\Users\jorda\Desktop\Code\Python\DRC\DRC_2026\Testing_Data\Test6"),
    Path(r"C:\Users\jorda\Desktop\Code\Python\DRC\DRC_2026\Testing_Data\Test7"),
]

OUTPUT_DIR = Path(r"C:\Users\jorda\Desktop\Code\Python\DRC\DRC_2026\Testing_Data\Test67")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

output_csv = OUTPUT_DIR / "labels.csv"

with output_csv.open("w", newline="") as f_out:
    writer = csv.writer(f_out)
    writer.writerow(["timestamp", "camera_id", "image", "mapped_value"])

    for source_dir in SOURCE_DIRS:
        source_csv = source_dir / "labels.csv"
        if not source_csv.exists():
            print(f"Skipping missing CSV: {source_csv}")
            continue

        dataset_name = source_dir.name

        with source_csv.open("r", newline="") as f_in:
            reader = csv.DictReader(f_in)
            for row in reader:
                old_image_name = row["image"]
                old_image_path = source_dir / old_image_name

                if not old_image_path.exists():
                    print(f"Missing image, skipping: {old_image_path}")
                    continue

                new_image_name = f"{dataset_name}_{old_image_name}"
                new_image_path = OUTPUT_DIR / new_image_name

                shutil.copy2(old_image_path, new_image_path)

                writer.writerow([
                    row["timestamp"],
                    row["camera_id"],
                    new_image_name,
                    row["mapped_value"]
                ])

print(f"Done. Merged dataset written to {OUTPUT_DIR}")