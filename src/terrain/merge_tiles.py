import sys
from pathlib import Path

# parents[1] -> src/  (allows: python src/terrain/merge_tiles.py)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rasterio
from rasterio.merge import merge

from paths import PROJECT_ROOT

data_dir = PROJECT_ROOT / "data" / "dem" / "siberia_unmerged"
# list of tif files to merge {place your tif files in dem folder, and add their filename in tif_files list}
tif_files = [
  # N59 Row
  "N59E090.tif", "N59E091.tif", "N59E092.tif", "N59E093.tif", "N59E094.tif", "N59E095.tif", "N59E096.tif", "N59E097.tif", "N59E098.tif", "N59E099.tif", "N59E100.tif",
  # N58 Row
  "N58E090.tif", "N58E091.tif", "N58E092.tif", "N58E093.tif", "N58E094.tif", "N58E095.tif", "N58E096.tif", "N58E097.tif", "N58E098.tif", "N58E099.tif", "N58E100.tif",
  # N57 Row
  "N57E090.tif", "N57E091.tif", "N57E092.tif", "N57E093.tif", "N57E094.tif", "N57E095.tif", "N57E096.tif", "N57E097.tif", "N57E098.tif", "N57E099.tif", "N57E100.tif",
  # N56 Row
  "N56E090.tif", "N56E091.tif", "N56E092.tif", "N56E093.tif", "N56E094.tif", "N56E095.tif", "N56E096.tif", "N56E097.tif", "N56E098.tif", "N56E099.tif", "N56E100.tif",
  # N55 Row
  "N55E090.tif", "N55E091.tif", "N55E092.tif", "N55E093.tif", "N55E094.tif", "N55E095.tif", "N55E096.tif", "N55E097.tif", "N55E098.tif", "N55E099.tif", "N55E100.tif",
  # N54 Row
  "N54E090.tif", "N54E091.tif", "N54E092.tif", "N54E093.tif", "N54E094.tif", "N54E095.tif", "N54E096.tif", "N54E097.tif", "N54E098.tif", "N54E099.tif", "N54E100.tif"
]

# path for each files
tif_paths = [data_dir / i for i in tif_files] # iterate and set path to each files

# check for missing files and fail fast with a clear message
missing = [p for p in tif_paths if not p.exists()]
if missing:
    print("The following DEM files are missing:")
    for m in missing:
        print(" -", m)
    raise SystemExit(1)

# open dataset
datasets = [rasterio.open(p) for p in tif_paths]

# merge files
merged_array, merged_transform = merge(datasets)

# get metadata from first file
out_meta = datasets[0].meta.copy()
out_meta.update({
    "driver": "GTiff",
    "height": merged_array.shape[1],
    "width": merged_array.shape[2],
    "transform": merged_transform
})

# save merged file {modify the desired filename}
output_path = data_dir / "merged_dem_sib_N59_E090_E099.tif"
with rasterio.open(output_path, "w", **out_meta) as dest:
    dest.write(merged_array)

# close datasets
for ds in datasets:
    ds.close()

print(f"  Merged DEM saved: {output_path}")
print(f"  Shape: {merged_array.shape[1]} x {merged_array.shape[2]}")
