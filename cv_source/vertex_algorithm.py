import pygame
from math import degrees, atan2

pygame.init()
screen = pygame.display.set_mode((640,480))

# COPY PASTE ---------------------------------------------------------
corners = [[115.0, 0.0], [104.0, 31.0], [234.0, 150.0], [280.0, 112.0], [180.0, 0.0]]
# --------------------------------------------------------------------

# Normalise
corners = [[x[0] * 2, 480 - x[1] * 2] for x in corners]
running = True

# ALGORITHM

# pick top two vertices
top = None
sto = None
bot = None
sbo = None

for corner in corners:
    if top == None:
        top = corner
    elif sto == None:
        if corner[1] >= top[1]:
            sto = top
            top = corner
    else:
        if corner[1] >= top[1]:
            sto = top
            top = corner
        elif corner[1] >= sto[1]:
            sto = corner

for corner in corners:
    if bot == None:
        bot = corner
    elif sbo == None:
        if corner[1] <= bot[1]:
            sbo = bot
            bot = corner
    else:
        if corner[1] <= bot[1]:
            sbo = bot
            bot = corner
        elif corner[1] <= sbo[1]:
            sbo = corner

# Pick right and left out of top
tl = None
tr = None
bl = None
br = None

if top == None or sto == None:
    running = False
    print("Couldn't find top two.")
else:
    if top[0] < sto[0]:
        tl = top
        tr = sto
    else:
        tl = sto
        tr = top

if bot == None or sbo == None:
    running = False
    print("Couldn't find bot two.")
else:
    if bot[0] < sbo[0]:
        bl = bot
        br = sbo
    else:
        bl = sbo
        br = bot

dx = (tl[0] - bl[0] + tr[0] - br[0]) * 0.5
dy = (tl[1] - bl[1] + tr[1] - br[1]) * 0.5

angle = degrees(atan2(dx, dy))
print(angle)

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((255,255,255))

    for corner in corners:
        pygame.draw.circle(screen, (0,0,0), (corner[0], corner[1]), 5)

    pygame.draw.circle(screen, (0,0,0), (tl[0], tl[1]), 10, 3)
    pygame.draw.circle(screen, (255,0,0), (tr[0], tr[1]), 10, 3)
    pygame.draw.circle(screen, (0,255,0), (bl[0], bl[1]), 10, 3)
    pygame.draw.circle(screen, (0,0,255), (br[0], br[1]), 10, 3)

    pygame.display.update()
pygame.quit()
