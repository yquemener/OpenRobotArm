"""A pyglet-based interactive 3D scene viewer.
"""
from OpenGL.GL import *
from OpenGL.GLU import *

import copy
import os
import sys
from threading import Thread, RLock
import time

import imageio
import numpy as np
import OpenGL
import pyrender
import trimesh
from urdfpy import URDF

try:
    from Tkinter import Tk, tkFileDialog as filedialog
except Exception:
    try:
        from tkinter import Tk, filedialog as filedialog
    except Exception:
        pass

from pyrender.constants import (TARGET_OPEN_GL_MAJOR, TARGET_OPEN_GL_MINOR,
                        MIN_OPEN_GL_MAJOR, MIN_OPEN_GL_MINOR,
                        TEXT_PADDING, DEFAULT_SCENE_SCALE,
                        DEFAULT_Z_FAR, DEFAULT_Z_NEAR, RenderFlags, TextAlign)
from pyrender.light import DirectionalLight
from pyrender.node import Node
from pyrender.camera import PerspectiveCamera, OrthographicCamera, IntrinsicsCamera
from pyrender.trackball import Trackball
from pyrender.renderer import Renderer
from pyrender.mesh import Mesh

import pyglet
from pyglet import clock
pyglet.options['shadow_window'] = False


class Viewer2(pyglet.window.Window):
    def __init__(self, scene, viewport_size=None,
                 render_flags=None, viewer_flags=None,
                 registered_keys=None, run_in_thread=False, **kwargs):

        #######################################################################
        # Save attributes and flags
        #######################################################################
        if viewport_size is None:
            viewport_size = (640, 480)
        self._scene = scene
        self._viewport_size = viewport_size
        self._render_lock = RLock()
        self._is_active = False
        self._should_close = False
        self._run_in_thread = run_in_thread

        self._default_render_flags = {
            'flip_wireframe': False,
            'all_wireframe': False,
            'all_solid': False,
            'shadows': False,
            'vertex_normals': False,
            'face_normals': False,
            'cull_faces': True,
            'point_size': 1.0,
        }
        self._default_viewer_flags = {
            'mouse_pressed': False,
            'rotate': False,
            'rotate_rate': np.pi / 3.0,
            'rotate_axis': np.array([0.0, 0.0, 1.0]),
            'view_center': None,
            'record': False,
            'use_raymond_lighting': False,
            'use_direct_lighting': False,
            'lighting_intensity': 3.0,
            'use_perspective_cam': True,
            'save_directory': None,
            'window_title': 'Scene Viewer',
            'refresh_rate': 30.0,
            'fullscreen': False,
            'show_world_axis': False,
            'show_mesh_axes': False,
            'caption': None
        }
        self._render_flags = self._default_render_flags.copy()
        self._viewer_flags = self._default_viewer_flags.copy()
        self._viewer_flags['rotate_axis'] = (
            self._default_viewer_flags['rotate_axis'].copy()
        )

        if render_flags is not None:
            self._render_flags.update(render_flags)
        if viewer_flags is not None:
            self._viewer_flags.update(viewer_flags)

        for key in kwargs:
            if key in self.render_flags:
                self._render_flags[key] = kwargs[key]
            elif key in self.viewer_flags:
                self._viewer_flags[key] = kwargs[key]

        # TODO MAC OS BUG FOR SHADOWS
        if sys.platform == 'darwin':
            self._render_flags['shadows'] = False

        self._registered_keys = {}
        if registered_keys is not None:
            self._registered_keys = {
                ord(k.lower()): registered_keys[k] for k in registered_keys
            }

        #######################################################################
        # Save internal settings
        #######################################################################

        # Set up caption stuff
        self._message_text = None
        self._ticks_till_fade = 2.0 / 3.0 * self.viewer_flags['refresh_rate']
        self._message_opac = 1.0 + self._ticks_till_fade

        # Set up raymond lights and direct lights
        self._raymond_lights = self._create_raymond_lights()
        self._direct_light = self._create_direct_light()

        # Set up axes
        self._axes = {}
        self._axis_mesh = Mesh.from_trimesh(
            trimesh.creation.axis(origin_size=0.1, axis_radius=0.05,
                                  axis_length=1.0), smooth=False)
        if self.viewer_flags['show_world_axis']:
            self._set_axes(world=self.viewer_flags['show_world_axis'],
                           mesh=self.viewer_flags['show_mesh_axes'])

        #######################################################################
        # Set up camera node
        #######################################################################
        self._camera_node = None
        self._prior_main_camera_node = None
        self._default_camera_pose = None
        self._default_persp_cam = None
        self._default_orth_cam = None
        self._trackball = None
        self._saved_frames = []

        # Extract main camera from scene and set up our mirrored copy
        znear = None
        zfar = None
        if scene.main_camera_node is not None:
            n = scene.main_camera_node
            camera = copy.copy(n.camera)
            if isinstance(camera, (PerspectiveCamera, IntrinsicsCamera)):
                self._default_persp_cam = camera
                znear = camera.znear
                zfar = camera.zfar
            elif isinstance(camera, OrthographicCamera):
                self._default_orth_cam = camera
                znear = camera.znear
                zfar = camera.zfar
            self._default_camera_pose = scene.get_pose(scene.main_camera_node)
            self._prior_main_camera_node = n

        # Set defaults as needed
        if zfar is None:
            zfar = max(scene.scale * 10.0, DEFAULT_Z_FAR)
        if znear is None or znear == 0:
            if scene.scale == 0:
                znear = DEFAULT_Z_NEAR
            else:
                znear = min(scene.scale / 10.0, DEFAULT_Z_NEAR)

        if self._default_persp_cam is None:
            self._default_persp_cam = PerspectiveCamera(
                yfov=np.pi / 3.0, znear=znear, zfar=zfar
            )
        if self._default_orth_cam is None:
            xmag = ymag = scene.scale
            if scene.scale == 0:
                xmag = ymag = 1.0
            self._default_orth_cam = OrthographicCamera(
                xmag=xmag, ymag=ymag,
                znear=znear,
                zfar=zfar
            )
        if self._default_camera_pose is None:
            self._default_camera_pose = self._compute_initial_camera_pose()

        # Pick camera
        if self.viewer_flags['use_perspective_cam']:
            camera = self._default_persp_cam
        else:
            camera = self._default_orth_cam

        self._camera_node = Node(
            matrix=self._default_camera_pose, camera=camera
        )
        scene.add_node(self._camera_node)
        scene.main_camera_node = self._camera_node

        #######################################################################
        # Initialize OpenGL context and renderer
        #######################################################################
        self._renderer = Renderer(
            self._viewport_size[0], self._viewport_size[1],
            self.render_flags['point_size']
        )
        self._is_active = True

        if self.run_in_thread:
            self._thread = Thread(target=self._init_and_start_app)
            self._thread.start()
        else:
            self._init_and_start_app()

    @property
    def scene(self):
        """:class:`.Scene` : The scene being visualized.
        """
        return self._scene

    @property
    def viewport_size(self):
        """(2,) int : The width and height of the viewing window.
        """
        return self._viewport_size

    @property
    def render_lock(self):
        """:class:`threading.RLock` : If acquired, prevents the viewer from
        rendering until released.

        Run :meth:`.Viewer.render_lock.acquire` before making updates to
        the scene in a different thread, and run
        :meth:`.Viewer.render_lock.release` once you're done to let the viewer
        continue.
        """
        return self._render_lock

    @property
    def is_active(self):
        """bool : `True` if the viewer is active, or `False` if it has
        been closed.
        """
        return self._is_active

    @property
    def run_in_thread(self):
        """bool : Whether the viewer was run in a separate thread.
        """
        return self._run_in_thread

    @property
    def render_flags(self):
        """dict : Flags for controlling the renderer's behavior.

        - ``flip_wireframe``: `bool`, If `True`, all objects will have their
          wireframe modes flipped from what their material indicates.
          Defaults to `False`.
        - ``all_wireframe``: `bool`, If `True`, all objects will be rendered
          in wireframe mode. Defaults to `False`.
        - ``all_solid``: `bool`, If `True`, all objects will be rendered in
          solid mode. Defaults to `False`.
        - ``shadows``: `bool`, If `True`, shadows will be rendered.
          Defaults to `False`.
        - ``vertex_normals``: `bool`, If `True`, vertex normals will be
          rendered as blue lines. Defaults to `False`.
        - ``face_normals``: `bool`, If `True`, face normals will be rendered as
          blue lines. Defaults to `False`.
        - ``cull_faces``: `bool`, If `True`, backfaces will be culled.
          Defaults to `True`.
        - ``point_size`` : float, The point size in pixels. Defaults to 1px.

        """
        return self._render_flags

    @render_flags.setter
    def render_flags(self, value):
        self._render_flags = value

    @property
    def viewer_flags(self):
        """dict : Flags for controlling the viewer's behavior.

        The valid keys for ``viewer_flags`` are as follows:

        - ``rotate``: `bool`, If `True`, the scene's camera will rotate
          about an axis. Defaults to `False`.
        - ``rotate_rate``: `float`, The rate of rotation in radians per second.
          Defaults to `PI / 3.0`.
        - ``rotate_axis``: `(3,) float`, The axis in world coordinates to
          rotate about. Defaults to ``[0,0,1]``.
        - ``view_center``: `(3,) float`, The position to rotate the scene
          about. Defaults to the scene's centroid.
        - ``use_raymond_lighting``: `bool`, If `True`, an additional set of
          three directional lights that move with the camera will be added to
          the scene. Defaults to `False`.
        - ``use_direct_lighting``: `bool`, If `True`, an additional directional
          light that moves with the camera and points out of it will be
          added to the scene. Defaults to `False`.
        - ``lighting_intensity``: `float`, The overall intensity of the
          viewer's additional lights (when they're in use). Defaults to 3.0.
        - ``use_perspective_cam``: `bool`, If `True`, a perspective camera will
          be used. Otherwise, an orthographic camera is used. Defaults to
          `True`.
        - ``save_directory``: `str`, A directory to open the file dialogs in.
          Defaults to `None`.
        - ``window_title``: `str`, A title for the viewer's application window.
          Defaults to `"Scene Viewer"`.
        - ``refresh_rate``: `float`, A refresh rate for rendering, in Hertz.
          Defaults to `30.0`.
        - ``fullscreen``: `bool`, Whether to make viewer fullscreen.
          Defaults to `False`.
        - ``show_world_axis``: `bool`, Whether to show the world axis.
          Defaults to `False`.
        - ``show_mesh_axes``: `bool`, Whether to show the individual mesh axes.
          Defaults to `False`.
        - ``caption``: `list of dict`, Text caption(s) to display on
          the viewer. Defaults to `None`.

        """
        return self._viewer_flags

    @viewer_flags.setter
    def viewer_flags(self, value):
        self._viewer_flags = value

    def on_close(self):
        """Exit the event loop when the window is closed.
        """
        # Remove our camera and restore the prior one
        if self._camera_node is not None:
            self.scene.remove_node(self._camera_node)
        if self._prior_main_camera_node is not None:
            self.scene.main_camera_node = self._prior_main_camera_node

        # Delete any lighting nodes that we've attached
        if self.viewer_flags['use_raymond_lighting']:
            for n in self._raymond_lights:
                if self.scene.has_node(n):
                    self.scene.remove_node(n)
        if self.viewer_flags['use_direct_lighting']:
            if self.scene.has_node(self._direct_light):
                self.scene.remove_node(self._direct_light)

        # Delete any axis nodes that we've attached
        self._remove_axes()

        # Delete renderer
        if self._renderer is not None:
            self._renderer.delete()
        self._renderer = None

        # Force clean-up of OpenGL context data
        try:
            OpenGL.contextdata.cleanupContext()
            self.close()
        except Exception:
            pass
        finally:
            self._is_active = False
            super(Viewer2, self).on_close()
            pyglet.app.exit()

    def on_draw(self):
        """Redraw the scene into the viewing window.
        """
        if self._renderer is None:
            return
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self._render()

    def on_resize(self, width, height):
        """Resize the camera and trackball when the window is resized.
        # """
        # if self._renderer is None:
        #     return
        #
        # self._viewport_size = (width, height)
        # self._trackball.resize(self._viewport_size)
        # self._renderer.viewport_width = self._viewport_size[0]
        # self._renderer.viewport_height = self._viewport_size[1]
        # self.on_draw()
        return


    @staticmethod
    def _time_event(dt, self):

        if self._should_close:
            self.on_close()
        else:
            self.on_draw()


    def _render(self):
        """Render the scene into the framebuffer and flip.
        """

        flags = RenderFlags.NONE
        self._renderer.render(self.scene, flags)

    def _init_and_start_app(self):
        # Try multiple configs starting with target OpenGL version
        # and multisampling and removing these options if exception
        # Note: multisampling not available on all hardware
        from pyglet.gl import Config
        confs = [Config(sample_buffers=1, samples=4,
                        depth_size=24,
                        double_buffer=True,
                        major_version=TARGET_OPEN_GL_MAJOR,
                        minor_version=TARGET_OPEN_GL_MINOR),
                 Config(depth_size=24,
                        double_buffer=True,
                        major_version=TARGET_OPEN_GL_MAJOR,
                        minor_version=TARGET_OPEN_GL_MINOR),
                 Config(sample_buffers=1, samples=4,
                        depth_size=24,
                        double_buffer=True,
                        major_version=MIN_OPEN_GL_MAJOR,
                        minor_version=MIN_OPEN_GL_MINOR),
                 Config(depth_size=24,
                        double_buffer=True,
                        major_version=MIN_OPEN_GL_MAJOR,
                        minor_version=MIN_OPEN_GL_MINOR)]
        for conf in confs:
            try:
                super(Viewer2, self).__init__(config=conf, resizable=True,
                                             width=self._viewport_size[0],
                                             height=self._viewport_size[1])
                break
            except pyglet.window.NoSuchConfigException:
                pass

        if not self.context:
            raise ValueError('Unable to initialize an OpenGL 3+ context')
        # clock.schedule_interval(
        #     Viewer2._time_event, 1.0 / self.viewer_flags['refresh_rate'], self
        # )
        # self.switch_to()
        # self.set_caption(self.viewer_flags['window_title'])
        pyglet.app.run()

    def _compute_initial_camera_pose(self):
        centroid = self.scene.centroid
        if self.viewer_flags['view_center'] is not None:
            centroid = self.viewer_flags['view_center']
        scale = self.scene.scale
        if scale == 0.0:
            scale = DEFAULT_SCENE_SCALE

        s2 = 1.0 / np.sqrt(2.0)
        cp = np.eye(4)
        cp[:3,:3] = np.array([
            [0.0, -s2, s2],
            [1.0, 0.0, 0.0],
            [0.0, s2, s2]
        ])
        hfov = np.pi / 6.0
        dist = scale / (2.0 * np.tan(hfov))
        cp[:3,3] = dist * np.array([1.0, 0.0, 1.0]) + centroid

        return cp

    def _create_raymond_lights(self):
        thetas = np.pi * np.array([1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0])
        phis = np.pi * np.array([0.0, 2.0 / 3.0, 4.0 / 3.0])

        nodes = []

        for phi, theta in zip(phis, thetas):
            xp = np.sin(theta) * np.cos(phi)
            yp = np.sin(theta) * np.sin(phi)
            zp = np.cos(theta)

            z = np.array([xp, yp, zp])
            z = z / np.linalg.norm(z)
            x = np.array([-z[1], z[0], 0.0])
            if np.linalg.norm(x) == 0:
                x = np.array([1.0, 0.0, 0.0])
            x = x / np.linalg.norm(x)
            y = np.cross(z, x)

            matrix = np.eye(4)
            matrix[:3,:3] = np.c_[x,y,z]
            nodes.append(Node(
                light=DirectionalLight(color=np.ones(3), intensity=1.0),
                matrix=matrix
            ))

        return nodes

    def _create_direct_light(self):
        light = DirectionalLight(color=np.ones(3), intensity=1.0)
        n = Node(light=light, matrix=np.eye(4))
        return n

    def _set_axes(self, world, mesh):
        scale = self.scene.scale
        if world:
            if 'scene' not in self._axes:
                n = Node(mesh=self._axis_mesh, scale=np.ones(3) * scale * 0.3)
                self.scene.add_node(n)
                self._axes['scene'] = n
        else:
            if 'scene' in self._axes:
                self.scene.remove_node(self._axes['scene'])
                self._axes.pop('scene')

        if mesh:
            old_nodes = []
            existing_axes = set([self._axes[k] for k in self._axes])
            for node in self.scene.mesh_nodes:
                if node not in existing_axes:
                    old_nodes.append(node)

            for node in old_nodes:
                if node in self._axes:
                    continue
                n = Node(
                    mesh=self._axis_mesh,
                    scale=np.ones(3) * node.mesh.scale * 0.5
                )
                self.scene.add_node(n, parent_node=node)
                self._axes[node] = n
        else:
            to_remove = set()
            for main_node in self._axes:
                if main_node in self.scene.mesh_nodes:
                    self.scene.remove_node(self._axes[main_node])
                    to_remove.add(main_node)
            for main_node in to_remove:
                self._axes.pop(main_node)

    def _remove_axes(self):
        for main_node in self._axes:
            axis_node = self._axes[main_node]
            self.scene.remove_node(axis_node)
        self._axes = {}

    def _location_to_x_y(self, location):
        if location == TextAlign.CENTER:
            return (self.viewport_size[0] / 2.0, self.viewport_size[1] / 2.0)
        elif location == TextAlign.CENTER_LEFT:
            return (TEXT_PADDING, self.viewport_size[1] / 2.0)
        elif location == TextAlign.CENTER_RIGHT:
            return (self.viewport_size[0] - TEXT_PADDING,
                    self.viewport_size[1] / 2.0)
        elif location == TextAlign.BOTTOM_LEFT:
            return (TEXT_PADDING, TEXT_PADDING)
        elif location == TextAlign.BOTTOM_RIGHT:
            return (self.viewport_size[0] - TEXT_PADDING, TEXT_PADDING)
        elif location == TextAlign.BOTTOM_CENTER:
            return (self.viewport_size[0] / 2.0, TEXT_PADDING)
        elif location == TextAlign.TOP_LEFT:
            return (TEXT_PADDING, self.viewport_size[1] - TEXT_PADDING)
        elif location == TextAlign.TOP_RIGHT:
            return (self.viewport_size[0] - TEXT_PADDING,
                    self.viewport_size[1] - TEXT_PADDING)
        elif location == TextAlign.TOP_CENTER:
            return (self.viewport_size[0] / 2.0,
                    self.viewport_size[1] - TEXT_PADDING)


robot = URDF.load('/home/yves/tmp/urdfpy/tests/data/ur5/braccio.urdf')

renderer = Renderer(800, 600)

fk = robot.visual_trimesh_fk()
scene = pyrender.Scene()
for tm in fk:
    pose = fk[tm]
    mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
    scene.add(mesh, pose=pose)

Viewer2(scene, use_direct_lighting=True)