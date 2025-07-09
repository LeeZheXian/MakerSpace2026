# MakerSpace2026


# YOLOv11 Training DataSet (from ZYOLO.pt, feel free to use it to further improve your model's performance)

```python
!pip install roboflow

from roboflow import Roboflow
rf = Roboflow(api_key="Lc5Zsror7KEAPwVHhhr4")
project = rf.workspace("eureka-c2uug").project("makerspace")
version = project.version(3)
dataset = version.download("yolov11")
```

# Train your model using TrainingYOLO.ipynb or using this link as reference: "https://colab.research.google.com/drive/11Nv1zIOYJO0zZKMIq6kAe7WQ8CIK08jP?usp=sharing"
