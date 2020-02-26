from pylibdmtx.pylibdmtx import encode, decode
from PIL import Image

encoded = encode('100:100:0'.encode('utf8'))    # pose (x,y,theta)
img = Image.frombytes('RGB', (encoded.width, encoded.height), encoded.pixels)
img.save('dmtx.png')
print(decode(Image.open('dmtx.png')))
