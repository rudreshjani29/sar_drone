"""
Phase 3.1 — Detection Under Degraded Conditions (static test images)

Instead of a binary "detected / not detected," this builds a detection
PERFORMANCE ENVELOPE: how a real pretrained person-detector's confidence
degrades under conditions that mirror real SAR bottlenecks —
altitude/resolution loss, canopy occlusion, and lighting.

Approach:
  1. Start from a base image containing a person (bundled YOLOv8 sample
     image — no external image sourcing needed).
  2. Programmatically generate degraded variants across three axes:
       - altitude (simulated via downscaling resolution, then upscaling
         back — mimics fewer pixels-on-target at higher altitude)
       - occlusion (simulated via semi-transparent overlay patches,
         mimicking canopy coverage)
       - lighting (simulated via brightness adjustment)
  3. Run YOLOv8 (pretrained on COCO, class 0 = person) on every variant.
  4. Record the best person-detection confidence for each — 0.0 if no
     person detected at all.
  5. Save results to CSV and plot the three degradation curves.

Requires: pip install ultralytics pillow matplotlib
"""

import csv
from pathlib import Path

from PIL import Image, ImageEnhance
from ultralytics import YOLO
import matplotlib.pyplot as plt


OUTPUT_DIR = Path("degraded_variants")
OUTPUT_DIR.mkdir(exist_ok=True)

RESULTS_CSV = "detection_results.csv"
PERSON_CLASS_ID = 0  # COCO class index for "person"


def get_base_image():
    """Uses YOLOv8's own bundled sample image (ships with the ultralytics
    package for testing/demos) so we don't need to source an external photo."""
    import ultralytics
    assets_dir = Path(ultralytics.__file__).parent / "assets"
    candidate = assets_dir / "bus.jpg"  # contains people, standard demo image
    if not candidate.exists():
        raise FileNotFoundError(
            f"Expected bundled sample image not found at {candidate}. "
            "Check your ultralytics installation, or point BASE_IMAGE to "
            "your own image containing a person."
        )
    return candidate


# ---------------------------------------------------------------------------
# DEGRADATION GENERATORS
# ---------------------------------------------------------------------------
def simulate_altitude(img, scale_fraction):
    """Downscale then upscale back to original size — approximates fewer
    pixels-on-target as if the camera were further away (higher altitude),
    without changing the overall frame composition."""
    w, h = img.size
    small = img.resize((max(1, int(w * scale_fraction)), max(1, int(h * scale_fraction))))
    return small.resize((w, h))


def simulate_occlusion(img, coverage_fraction):
    """Overlays a semi-transparent patch centered on the frame, sized
    proportionally to coverage_fraction. Centered (rather than top-down)
    so it actually overlaps subjects near the middle of frame — a
    top-down strip can miss the subject entirely and produce a
    meaningless result."""
    if coverage_fraction == 0:
        return img.copy()
    overlay = Image.new("RGBA", img.size, (60, 90, 40, 0))
    w, h = img.size
    # Patch grows from the center outward in both dimensions as coverage increases
    patch_w = int(w * coverage_fraction)
    patch_h = int(h * coverage_fraction)
    x0 = (w - patch_w) // 2
    y0 = (h - patch_h) // 2
    from PIL import ImageDraw
    draw = ImageDraw.Draw(overlay)
    draw.rectangle([x0, y0, x0 + patch_w, y0 + patch_h], fill=(60, 90, 40, 250))
    base_rgba = img.convert("RGBA")
    combined = Image.alpha_composite(base_rgba, overlay)
    return combined.convert("RGB")


def simulate_lighting(img, brightness_factor):
    """brightness_factor: 1.0 = unchanged, <1.0 = darker (dusk/night-like),
    >1.0 = brighter/washed out."""
    enhancer = ImageEnhance.Brightness(img)
    return enhancer.enhance(brightness_factor)


# ---------------------------------------------------------------------------
# DETECTION
# ---------------------------------------------------------------------------
def best_person_confidence(model, img):
    """Runs detection on a PIL image, returns the highest confidence score
    among 'person' detections, or 0.0 if none found."""
    results = model.predict(img, verbose=False)
    result = results[0]
    if result.boxes is None or len(result.boxes) == 0:
        return 0.0
    person_confs = [
        float(conf) for conf, cls in zip(result.boxes.conf, result.boxes.cls)
        if int(cls) == PERSON_CLASS_ID
    ]
    return max(person_confs) if person_confs else 0.0


def main():
    print("-- Loading YOLOv8 (pretrained on COCO)...")
    model = YOLO("yolov8n.pt")  # nano variant — fast, fine for this test

    base_path = get_base_image()
    base_img = Image.open(base_path).convert("RGB")
    print(f"-- Base image: {base_path}")

    rows = []

    # --- Axis 1: Altitude (resolution loss) ---
    print("\n-- Testing altitude/resolution degradation...")
    for scale in [1.0, 0.25, 0.1, 0.05, 0.03, 0.02]:
        variant = simulate_altitude(base_img, scale)
        variant_path = OUTPUT_DIR / f"altitude_scale_{scale}.jpg"
        variant.save(variant_path)
        conf = best_person_confidence(model, variant)
        print(f"   scale={scale:.4f} (~{int(1/scale) if scale else 0}x zoomed out) "
              f"-> confidence={conf:.3f}")
        rows.append({"axis": "altitude", "condition": scale, "confidence": conf})

    # --- Axis 2: Occlusion (canopy coverage) ---
    print("\n-- Testing occlusion degradation...")
    for coverage in [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]:
        variant = simulate_occlusion(base_img, coverage)
        variant_path = OUTPUT_DIR / f"occlusion_{coverage}.jpg"
        variant.save(variant_path)
        conf = best_person_confidence(model, variant)
        print(f"   coverage={coverage:.2f} -> confidence={conf:.3f}")
        rows.append({"axis": "occlusion", "condition": coverage, "confidence": conf})

    # --- Axis 3: Lighting ---
    print("\n-- Testing lighting degradation...")
    for brightness in [1.0, 0.3, 0.15, 0.08, 0.04, 0.02]:
        variant = simulate_lighting(base_img, brightness)
        variant_path = OUTPUT_DIR / f"lighting_{brightness}.jpg"
        variant.save(variant_path)
        conf = best_person_confidence(model, variant)
        print(f"   brightness={brightness:.2f} -> confidence={conf:.3f}")
        rows.append({"axis": "lighting", "condition": brightness, "confidence": conf})

    # --- Save results ---
    with open(RESULTS_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["axis", "condition", "confidence"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"\n-- Results saved to {RESULTS_CSV}")
    print(f"-- Degraded image variants saved to {OUTPUT_DIR}/")

    plot_envelope(rows)


def plot_envelope(rows):
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    axis_titles = {
        "altitude": ("Simulated Altitude (resolution scale)", "Scale factor (1.0 = full res)"),
        "occlusion": ("Simulated Canopy Occlusion", "Fraction of frame occluded"),
        "lighting": ("Simulated Lighting", "Brightness factor (1.0 = normal)"),
    }

    for ax, (axis_name, (title, xlabel)) in zip(axes, axis_titles.items()):
        xs = [r["condition"] for r in rows if r["axis"] == axis_name]
        ys = [r["confidence"] for r in rows if r["axis"] == axis_name]
        ax.plot(xs, ys, 'o-', color='tab:blue')
        ax.axhline(0.5, color='gray', linestyle='--', alpha=0.5, label='0.5 confidence threshold')
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel("Detection confidence")
        ax.set_ylim(-0.05, 1.05)
        ax.legend()

    plt.tight_layout()
    plt.savefig("detection_envelope.png", dpi=150)
    print("-- Envelope plot saved to detection_envelope.png")
    plt.show()


if __name__ == "__main__":
    main()
