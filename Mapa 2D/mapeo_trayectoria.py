import pygame
import math

pygame.init()

width, height=1200,1000

screen=pygame.display.set_mode((width,height))
pygame.display.set_caption("Trayectoria del ROVER")
clock=pygame.time.Clock()

scale=50  # 1 metro=50 pixeles

offset_x=width//2
offset_y=height//2

x=0
y=0
theta=0

v=0.5

omega=0.5

dt=0.05


path=[]

running=True

while running:
    clock.tick(60)
    
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            running=False
            
    # Simulación de movimiento

    x+=v*math.cos(theta)*dt
    y+=v*math.sin(theta)*dt
    theta+=omega*dt
    
    path.append((x,y))
    
    # Dibujar
    
    screen.fill((30,30,30))
    
    if len(path)>1:
        for i in range (1,len(path)):
            x1=path[i-1][0]*scale+offset_x
            y1=-path[i-1][1]*scale+offset_y
            x2=path[i][0]*scale+offset_x
            y2=-path[i][1]*scale+offset_y
            pygame.draw.line(screen,(0,255,0),(x1,y1),(x2,y2),2)
            
    rover_size=0.5 #metros
    
    points=[(rover_size,0),(-rover_size/2,rover_size/2),(-rover_size/2,-rover_size/2)]
    
    rotated_points=[]
    for px,py in points:
        rx=px*math.cos(theta)-py*math.sin(theta)
        ry=px*math.sin(theta)+py*math.cos(theta)
        
        rotated_points.append((rx*scale+x*scale+offset_x,-ry*scale-y*scale+offset_y))
        
    pygame.draw.polygon(screen,(255,0,0),rotated_points)
    
    pygame.display.flip()

pygame.quit()
    