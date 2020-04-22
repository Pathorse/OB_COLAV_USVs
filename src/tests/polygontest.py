from PIL import Image, ImageDraw
# Test

verts = generatePolygon( ctrX=250, ctrY=250, aveRadius=100, irregularity=0.8, spikeyness=0.0, numVerts=6 )

black = (0,0,0)
white=(255,255,255)
im = Image.new('RGB', (500, 500), white)
imPxAccess = im.load()
draw = ImageDraw.Draw(im)
tupVerts = map(tuple,verts)

# either use .polygon(), if you want to fill the area with a solid colour
draw.polygon(verts, outline=black,fill=white)

# or .line() if you want to control the line thickness, or use both methods together!
#draw.line( tupVerts+[tupVerts[0]], width=2, fill=black )

im.show()
