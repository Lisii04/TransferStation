import torch
import torch.nn
import onnx
from models import LeNet5


model_dict=torch.load("best.pt",map_location=torch.device('cpu'))
model=LeNet5()
model.load_state_dict(model_dict)
model.eval()

input_names = ['input']
output_names = ['output']

x = torch.randn(1, 1, 36, 36, requires_grad=True)

torch.onnx.export(model, x, 'best.onnx', input_names=input_names, output_names=output_names, verbose='True')
