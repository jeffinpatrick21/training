from ultralytics import YOLO

if __name__ == '__main__':
    model = YOLO('yolo11n.pt')
    model.train(data='data.yaml', epochs=600, device=0)
