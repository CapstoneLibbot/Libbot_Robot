# Patrol leaning-book detector

The real robot uses the IMX500 AI Camera for local frame classification. It classifies a full shelf view as `NORMAL_SHELF` or `LEANING_PRESENT`; it does not identify individual book boxes. A detection is reported at confidence `0.70` or higher for three consecutive frames. A new detection episode is enabled after two seconds without a positive frame. One JPEG from each episode is uploaded with its path and waypoint.

## Capture labeled frames

Run the camera service on the Pi, then capture images into two class folders. Collect varied shelves, distances, lighting, camera angles, normal book arrangements, and several degrees of leaning. Avoid putting near-duplicate frames in both train and validation data. Ask a librarian to label the frame based on whether a book needs attention.

```bash
python3 scripts/capture_patrol_dataset.py --label NORMAL_SHELF --count 500 --output patrol-dataset
python3 scripts/capture_patrol_dataset.py --label LEANING_PRESENT --count 500 --output patrol-dataset
```

The output layout is compatible with `torchvision.datasets.ImageFolder`. Split data by shelf/session before final evaluation when possible; random frame splits can overstate accuracy when adjacent frames are very similar.

## Train and prepare the IMX500 model

Train on a workstation or development machine, not on the robot:

```bash
python3 -m venv .venv-patrol
source .venv-patrol/bin/activate
python -m pip install torch torchvision onnx
python tools/train_patrol_classifier.py patrol-dataset --output patrol-model --epochs 12
```

The training tool saves `patrol_mobilenet_v2.pth`, an ONNX export, and `labels.json`. This is a starter pipeline: review the validation results and evaluate on a separate shelf/session set before deployment. The model must produce two classification scores in the same order as `labels.json`.

Use Raspberry Pi's official [AI Camera model deployment guide](https://www.raspberrypi.com/documentation/accessories/ai-camera.html#model-deployment) and Edge-MDT to quantize/compress the trained MobileNetV2 and convert it to IMX500 packer input. Then, on Raspberry Pi OS, package the converted output with:

```bash
sudo apt install imx500-all imx500-tools
imx500-package -i <Edge-MDT-packer-input.zip> -o <output-folder>
```

Copy the resulting `.rpk` to the Pi and copy `labels.json` alongside it using the name `<model>.rpk.labels.json` (for example, `patrol.rpk.labels.json`). The official [IMX500 model zoo](https://github.com/raspberrypi/imx500-models) is useful for reference, but its generic ImageNet classifier is not trained to detect leaning books.

Configure `/etc/libot/imx500-camera.env`:

```dotenv
LIBOT_PATROL_MODEL_RPK=/opt/libot/models/patrol.rpk
LIBOT_PATROL_LABELS=/opt/libot/models/patrol.rpk.labels.json
```

Restart and check `systemctl status libot-imx500-camera`. A real `LIBRARY_PATROL` dispatch now fails preflight with an explicit detector/model error until the RPK and labels are installed. Camera scanning for other workflows remains available without this model.
