#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

camera_location = [0.0, 0.0, 0.0]
camera_quat = Quaternion()
camera_quat.w = 1.0
azimuth = 0.0
elevation = 0.0

def hQuit():
  pygame.quit()
  quit()

def hMouseDown(event):
  #print('Mouse Down: {}'.format(event.button))
  pass

def hMouseMove(event):
  SCALE_FACTOR = 0.5
  global elevation
  global azimuth
  if event.buttons[0] == True:
    elevation += SCALE_FACTOR * event.rel[1]
    azimuth += SCALE_FACTOR * event.rel[0]
    #camera_location[1] += SCALE_FACTOR * event.rel[1]
    #print("{}".format(camera_location))
  #print('Mouse Move: {}'.format(event.rel))

def hKeyPress(event):
  SCALE_FACTOR = 0.1
  if event.key == K_w:
    camera_location[2] += SCALE_FACTOR
  elif event.key == K_a:
    camera_location[0] -= SCALE_FACTOR
  elif event.key == K_s:
    camera_location[2] -= SCALE_FACTOR
  elif event.key == K_d:
    camera_location[0] += SCALE_FACTOR
  elif event.key == K_q:
    camera_location[1] += SCALE_FACTOR
  elif event.key == K_e:
    camera_location[1] -= SCALE_FACTOR
  else:
    pass

def PrintText(txtStr, x, y):
  glPushMatrix()
  glLoadIdentity()
  glWindowPos2f(x, y)
  line = 0
  for txtChar in txtStr:
    if txtChar == '\n':
      line += 1
      glWindowPos2i(x, y - (line * 18))
    else:
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, ord(txtChar))
  glPopMatrix()

def callback(data):
  camera_location[0] = data.pose.position.x
  camera_location[1] = data.pose.position.y
  camera_location[2] = data.pose.position.z

  camera_quat.x = data.pose.orientation.x
  camera_quat.y = data.pose.orientation.y
  camera_quat.z = data.pose.orientation.z
  camera_quat.w = data.pose.orientation.w


def Cube():
  glBegin(GL_LINES)
  for xline in range(-20, 21):
    glVertex3f(xline, -1, -20)
    glVertex3f(xline, -1, 20)

  for yline in range(-20, 21):
    glVertex3f(-20, -1, yline)
    glVertex3f(20, -1, yline)
  glEnd()

def QuatToMat(quat):
  mat = [ 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0]

  mat[0] = quat.w**2 + quat.x**2 - quat.y**2 - quat.z**2
  mat[1] = 2 * quat.x * quat.y + 2 * quat.w * quat.z
  mat[2] = 2 * quat.x * quat.z - 2 * quat.w * quat.y

  mat[4] = 2 * quat.x * quat.y - 2 * quat.w * quat.z
  mat[5] = quat.w**2 - quat.x**2 + quat.y**2 - quat.z**2
  mat[6] = 2 * quat.y * quat.z + 2 * quat.w * quat.x

  mat[8] = 2 * quat.x * quat.z + 2 * quat.w * quat.y
  mat[9] = 2 * quat.y * quat.z - 2 * quat.w * quat.x
  mat[10] = quat.w**2 - quat.x**2 - quat.y**2 + quat.z**2

  return mat

def main():

  rospy.init_node('justaname')


  glutInit()
  pygame.init()
  display = (800,600)
  screen = pygame.display.set_mode(display, HWSURFACE | DOUBLEBUF | OPENGL)

  FPS = 60
  pgclock = pygame.time.Clock()
  pygame.key.set_repeat(1, int(1000/FPS))

  glMatrixMode(GL_PROJECTION)
  gluPerspective(45, (display[0]/display[1]),  0.1, 50.0)

  glTranslate(*camera_location)

  rospy.Subscriber('stereo_odometer/pose', PoseStamped, callback)

  while True:
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        hQuit()
      elif event.type == pygame.MOUSEMOTION:
        hMouseMove(event)
      elif event.type == pygame.KEYDOWN:
	pass
        #hKeyPress(event)
      else:
        #print("Unhandled Event")
        pass

    #glRotatef(1, 3, 1, 1)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    Cube()
    glLoadIdentity()
    gluPerspective(45, (display[0]/display[1]),  0.1, 50.0)
    corrcampos = [camera_location[0] * -1.0, camera_location[1] * 1.0, camera_location[2]]
    #glRotate(elevation, 1.0, 0, 0)
    #glRotate(azimuth, 0, 1.0, 0)
    glMultMatrixf(QuatToMat(camera_quat))
    #print(camera_quat)
    glTranslatef(*corrcampos)
    PrintText('X: {:6.2f}\nY:{:6.2f}\nZ: {:6.2f}'.format(camera_location[0], camera_location[1], camera_location[2]), 18, (display[1] - 18))
    PrintText('X: {:6.4f}\nY:{:6.4f}\nZ: {:6.4f}\nW: {:6.4f}'.format(camera_quat.x, camera_quat.y, camera_quat.z, camera_quat.w), 128, (display[1] - 18))
    pygame.display.flip()
    pgclock.tick(FPS)

    #print("{0:.2f}".format(camera_location[0]))

main()



