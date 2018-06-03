#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

import glfw
from OpenGL.GL import *
from OpenGL.GLU import *

camera_location = [0.0, 0.0, 0.0]
azimuth = 0.0
elevation = 0.0

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
      pass
  glPopMatrix()

def callback(data):
  camera_location[0] = data.pose.position.x
  camera_location[1] = data.pose.position.y
  camera_location[2] = data.pose.position.z


def Grid():
  glBegin(GL_LINES)
  for xline in range(-20, 21):
    glVertex3f(xline, -1, -20)
    glVertex3f(xline, -1, 20)

  for yline in range(-20, 21):
    glVertex3f(-20, -1, yline)
    glVertex3f(20, -1, yline)
  glEnd()

def main():

  rospy.init_node('justaname')

  if not glfw.init():
    return

  glfw.window_hint(glfw.SAMPLES, 4)
  window = glfw.create_window(640, 480, "Arena HUD", None, None)
  if not window:
    glfw.terminate()
    return

  glfw.make_context_current(window)

  glEnable(GL_MULTISAMPLE)
  glMatrixMode(GL_PROJECTION)
  gluPerspective(45, (640/480),  0.1, 50.0)
  glEnable(GL_DEPTH_TEST)
  glEnable(GL_BLEND)
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
  glEnable(GL_LINE_SMOOTH)
  glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE)

  glTranslate(*camera_location)

  rospy.Subscriber('stereo_odometer/pose', PoseStamped, callback)

  while not glfw.window_should_close(window):

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    Grid()
    glLoadIdentity()
    gluPerspective(45, (640/480),  0.1, 50.0)
    corrcampos = [camera_location[0] * -1.0, camera_location[1] * 1.0, camera_location[2]]
    glRotate(elevation, 1.0, 0, 0)
    glRotate(azimuth, 0, 1.0, 0)
    glTranslatef(*corrcampos)
    #PrintText('X: {:6.2f}\nY:{:6.2f}\nZ: {:6.2f}\nAzi: {:6.2f}\nEle: {:6.2f}'.format(camera_location[0], camera_location[1], camera_location[2], azimuth, elevation), 18, (display[1] - 18))

    glfw.swap_buffers(window)
    glfw.poll_events()

  glfw.terminate()

if __name__ == "__main__":
  main()



