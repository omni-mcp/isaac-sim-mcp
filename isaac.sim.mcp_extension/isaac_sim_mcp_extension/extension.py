"""
MIT License

Copyright (c) 2023-2025 omni-mcp

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

"""Extension module for Isaac Sim MCP."""

import asyncio
import carb
# import omni.ext
# import omni.ui as ui
import omni.usd
import threading
import time
import socket
import json
import traceback

import gc
from pxr import Usd, UsdGeom, Sdf, Gf

import omni
import omni.kit.commands
import omni.physx as _physx
import omni.timeline
from typing import Dict, Any, List, Optional, Union
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.prims import XFormPrim
import numpy as np
from omni.isaac.core import World
# Import Beaver3d and USDLoader
from isaac_sim_mcp_extension.gen3d import Beaver3d
from isaac_sim_mcp_extension.usd import USDLoader
from isaac_sim_mcp_extension.usd import USDSearch3d
import requests

# Extension Methods required by Omniverse Kit
# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class MCPExtension(omni.ext.IExt):
    def __init__(self) -> None:
        """Initialize the extension."""
        super().__init__()
        self.ext_id = None
        self.running = False
        self.host = None
        self.port = None
        self.socket = None
        self.server_thread = None
        self._usd_context = None
        self._physx_interface = None
        self._timeline = None
        self._window = None
        self._status_label = None
        self._server_thread = None
        self._models = None
        self._settings = carb.settings.get_settings()
        self._image_url_cache = {} # cache for image url
        self._text_prompt_cache = {} # cache for text prompt
        

    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""
        print("trigger  on_startup for: ", ext_id)
        print("settings: ", self._settings.get("/exts/omni.kit.pipapi"))
        self.port = self._settings.get("/exts/isaac.sim.mcp/server, port") or 8766
        self.host = self._settings.get("/exts/isaac.sim.mcp/server.host") or "localhost"
        if not hasattr(self, 'running'):
            self.running = False

        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()
        # omni.kit.commands.execute("CreatePrim", prim_type="Sphere")

        # print("sphere created")
        # result = self.execute_script('omni.kit.commands.execute("CreatePrim", prim_type="Cube")')
        # print("script executed", result)  
        self._start()
        # result = self.execute_script('omni.kit.commands.execute("CreatePrim", prim_type="Cube")')
        # print("script executed", result)  
    
    def on_shutdown(self):
        print("trigger  on_shutdown for: ", self.ext_id)
        self._models = {}
        gc.collect()
        self._stop()
    
    def _start(self):
        if self.running:
            print("Server is already running")
            return
            
        self.running = True
        
        try:
            # Create socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.listen(1)
            
            # Start server thread
            self.server_thread = threading.Thread(target=self._server_loop)
            self.server_thread.daemon = True
            self.server_thread.start()
            
            print(f"Isaac Sim MCP server started on {self.host}:{self.port}")
        except Exception as e:
            print(f"Failed to start server: {str(e)}")
            self.stop()
            
    def _stop(self):
        self.running = False
        
        # Close socket
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        
        # Wait for thread to finish
        if self.server_thread:
            try:
                if self.server_thread.is_alive():
                    self.server_thread.join(timeout=1.0)
            except:
                pass
            self.server_thread = None
        
        print("Isaac Sim MCP server stopped")

    def _server_loop(self):
        """Main server loop in a separate thread"""
        print("Server thread started")
        self.socket.settimeout(1.0)  # Timeout to allow for stopping
        if not hasattr(self, 'running'):
            self.running = False

        while self.running:
            try:
                # Accept new connection
                try:
                    client, address = self.socket.accept()
                    print(f"Connected to client: {address}")
                    
                    # Handle client in a separate thread
                    client_thread = threading.Thread(
                        target=self._handle_client,
                        args=(client,)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                except socket.timeout:
                    # Just check running condition
                    continue
                except Exception as e:
                    print(f"Error accepting connection: {str(e)}")
                    time.sleep(0.5)
            except Exception as e:
                print(f"Error in server loop: {str(e)}")
                if not self.running:
                    break
                time.sleep(0.5)
        
        print("Server thread stopped")
    
    def _handle_client(self, client):
        """Handle connected client"""
        print("Client handler started")
        client.settimeout(None)  # No timeout
        buffer = b''
        
        try:
            while self.running:
                # Receive data
                try:
                    data = client.recv(16384)
                    if not data:
                        print("Client disconnected")
                        break
                    
                    buffer += data
                    try:
                        # Try to parse command
                        command = json.loads(buffer.decode('utf-8'))
                        buffer = b''
                        
                        # Execute command in Isaac Sim's main thread
                        async def execute_wrapper():
                            try:
                                response = self.execute_command(command)
                                response_json = json.dumps(response)
                                print("response_json: ", response_json)
                                try:
                                    client.sendall(response_json.encode('utf-8'))
                                except:
                                    print("Failed to send response - client disconnected")
                            except Exception as e:
                                print(f"Error executing command: {str(e)}")
                                traceback.print_exc()
                                try:
                                    error_response = {
                                        "status": "error",
                                        "message": str(e)
                                    }
                                    client.sendall(json.dumps(error_response).encode('utf-8'))
                                except:
                                    pass
                            return None
                        # import omni.kit.commands
                        # import omni.kit.async
                        from omni.kit.async_engine import run_coroutine
                        task = run_coroutine(execute_wrapper())
                        # import asyncio
                        # asyncio.ensure_future(execute_wrapper())
                        #time.sleep(30)
                        
    
                        # 
                        # omni.kit.async.get_event_loop().create_task(create_sphere_async())
                        # TODO:Schedule execution in main thread
                        # bpy.app.timers.register(execute_wrapper, first_interval=0.0)
                        # omni.kit.app.get_app().post_to_main_thread(execute_wrapper())
                        # carb.apputils.get_app().get_update_event_loop().post(execute_wrapper)

                        # from omni.kit.async_engine import run_coroutine
                        # run_coroutine(execute_wrapper())
                        # omni.kit.app.get_app().get_update_event_stream().push(0, 0, {"fn": execute_wrapper})
                    except json.JSONDecodeError:
                        # Incomplete data, wait for more
                        pass
                except Exception as e:
                    print(f"Error receiving data: {str(e)}")
                    break
        except Exception as e:
            print(f"Error in client handler: {str(e)}")
        finally:
            try:
                client.close()
            except:
                pass
            print("Client handler stopped")

    # TODO: This is a temporary function to execute commands in the main thread
    def execute_command(self, command):
        """Execute a command in the main thread"""
        try:
            cmd_type = command.get("type")
            params = command.get("params", {})
            
            # TODO: Ensure we're in the right context
            if cmd_type in ["create_object", "modify_object", "delete_object"]:
                self._usd_context = omni.usd.get_context()
                self._execute_command_internal(command)
            else:
                return self._execute_command_internal(command)
                
        except Exception as e:
            print(f"Error executing command: {str(e)}")
            traceback.print_exc()
            return {"status": "error", "message": str(e)}

    def _execute_command_internal(self, command):
        """Internal command execution with proper context"""
        cmd_type = command.get("type")
        params = command.get("params", {})

        #todo: add a handler for extend simulation method if necessary
        handlers = {
            # "get_scene_info": self.get_scene_info,
            # "create_object": self.create_object,
            # "modify_object": self.modify_object,
            # "delete_object": self.delete_object,
            # "get_object_info": self.get_object_info,
            "execute_script": self.execute_script,
            "get_scene_info": self.get_scene_info,
            "omini_kit_command": self.omini_kit_command,
            "create_physics_scene": self.create_physics_scene,
            "create_robot": self.create_robot,
            "generate_3d_from_text_or_image": self.generate_3d_from_text_or_image,
            "transform": self.transform,
            "search_3d_usd_by_text": self.search_3d_usd_by_text,
            "list_prims": self.list_prims,
            "open_usd": self.open_usd,
            "import_usd": self.import_usd,
            "get_object_info": self.get_object_info,
            "move_prim": self.move_prim,
        }
        
        handler = handlers.get(cmd_type)
        if handler:
            try:
                print(f"Executing handler for {cmd_type}")
                result = handler(**params)
                print(f"Handler execution complete: /n", result)
                # return result
                if result and result.get("status") == "success":   
                    return {"status": "success", "result": result}
                else:
                    return {"status": "error", "message": result.get("message", "Unknown error")}
            except Exception as e:
                print(f"Error in handler: {str(e)}")
                traceback.print_exc()
                return {"status": "error", "message": str(e)}
        else:
            return {"status": "error", "message": f"Unknown command type: {cmd_type}"}
        

    

    def execute_script(self, code: str) :
        """Execute a Python script within the Isaac Sim context.
        
        Args:
            code: The Python script to execute.
            
        Returns:
            Dictionary with execution result.
        """
        try:
            # Create a local namespace
            local_ns = {}
            
            # Add frequently used modules to the namespace
            local_ns["omni"] = omni
            local_ns["carb"] = carb
            local_ns["Usd"] = Usd
            local_ns["UsdGeom"] = UsdGeom
            local_ns["Sdf"] = Sdf
            local_ns["Gf"] = Gf
            # code = script["code"]
            
            # Execute the script
            exec(code,  local_ns)
            
            # Get the result if any
            # result = local_ns.get("result", None)
            result = None
            
            
            return {
                "status": "success",
                "message": "Script executed successfully",
                "result": result
            }
        except Exception as e:
            carb.log_error(f"Error executing script: {e}")
            import traceback
            carb.log_error(traceback.format_exc())
            return {
                "status": "error",
                "message": str(e),
                "traceback": traceback.format_exc()
            }
        
    def get_scene_info(self):
        self._stage = omni.usd.get_context().get_stage()
        assert self._stage is not None
        stage_path = self._stage.GetRootLayer().realPath
        assets_root_path = get_assets_root_path()
        return {"status": "success", "message": "pong", "assets_root_path": assets_root_path}
        
    def omini_kit_command(self,  command: str, prim_type: str) -> Dict[str, Any]:
        omni.kit.commands.execute(command, prim_type=prim_type)
        print("command executed")
        return {"status": "success", "message": "command executed"}
    
    def create_robot(self, robot_type: str = "g1", position: List[float] = [0, 0, 0]):
        from omni.isaac.core.utils.prims import create_prim
        from omni.isaac.core.utils.stage import add_reference_to_stage, is_stage_loading
        from omni.isaac.nucleus import get_assets_root_path
        

        stage = omni.usd.get_context().get_stage()
        assets_root_path = get_assets_root_path()
        print("position: ", position)
        
        if robot_type.lower() == "franka":
            asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
            add_reference_to_stage(asset_path, "/Franka")
            robot_prim = XFormPrim(prim_path="/Franka")
            robot_prim.set_world_pose(position=np.array(position))
            return {"status": "success", "message": f"{robot_type} robot created"}
        elif robot_type.lower() == "jetbot":
            asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
            add_reference_to_stage(asset_path, "/Jetbot")
            robot_prim = XFormPrim(prim_path="/Jetbot")
            robot_prim.set_world_pose(position=np.array(position))
            return {"status": "success", "message": f"{robot_type} robot created"}
        elif robot_type.lower() == "carter":
            asset_path = assets_root_path + "/Isaac/Robots/Carter/carter.usd"
            add_reference_to_stage(asset_path, "/Carter")
            robot_prim = XFormPrim(prim_path="/Carter")
            robot_prim.set_world_pose(position=np.array(position))
            return {"status": "success", "message": f"{robot_type} robot created"}
        elif robot_type.lower() == "g1":
            asset_path = assets_root_path + "/Isaac/Robots/Unitree/G1/g1.usd"
            add_reference_to_stage(asset_path, "/G1")
            robot_prim = XFormPrim(prim_path="/G1")
            robot_prim.set_world_pose(position=np.array(position))
            return {"status": "success", "message": f"{robot_type} robot created"}
        elif robot_type.lower() == "go1":
            asset_path = assets_root_path + "/Isaac/Robots/Unitree/Go1/go1.usd"
            add_reference_to_stage(asset_path, "/Go1")
            robot_prim = XFormPrim(prim_path="/Go1")
            robot_prim.set_world_pose(position=np.array(position))
            return {"status": "success", "message": f"{robot_type} robot created"}
        else:
            # Default to Franka if unknown robot type
            asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
            add_reference_to_stage(asset_path, "/Franka")
            robot_prim = XFormPrim(prim_path="/Franka")
            robot_prim.set_world_pose(position=np.array(position))
            return {"status": "success", "message": f"{robot_type} robot created"}
    
    def create_physics_scene(
            self,
            objects: List[Dict[str, Any]] = [],
            floor: bool = True,
            gravity: List[float] = (0.0, -9.81, 0.0),
            scene_name: str = "None"
        ) -> Dict[str, Any]:
            """Create a physics scene with multiple objects."""
            try:
                # Set default values
                gravity = gravity or [0, -9.81, 0]
                scene_name = scene_name or "physics_scene"
                
                
                # Create a new stage
                #omni.kit.commands.execute("CreateNewStage")
                
                
                stage = omni.usd.get_context().get_stage()
                print("stage: ", stage)
                
                # print("start to create new sphere")
                # # import omni.kit.commands
                # omni.kit.commands.execute("CreatePrim", prim_type="Sphere")
                # print("create sphere successfully")
                
                # Set up the physics scene
                scene_path = "/World/PhysicsScene"
                omni.kit.commands.execute(
                    "CreatePrim",
                    prim_path=scene_path,
                    prim_type="PhysicsScene",
                    
                )
                #attributes={"physxScene:enabled": True , "physxScene:gravity": gravity},
                

                # Initialize simulation context with physics
                # simulation_context = SimulationContext()
                # my_world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
        
                # # Make sure the world is playing before initializing the robot
                # if not my_world.is_playing():
                #     my_world.play()
                #     # Wait a few frames for physics to stabilize
                # for _ in range(1000):
                #     my_world.step_async()
                # my_world.initialize_physics()

                # print("created physics scene: ", scene_path)
                
                # Create the World prim as a Xform
                world_path = "/World"
                omni.kit.commands.execute(
                    "CreatePrim",
                    prim_path=world_path,
                    prim_type="Xform",
                )
                print("create world: ", world_path)
                # Create a ground plane if requested
                if floor:
                    floor_path = "/World/ground"
                    omni.kit.commands.execute(
                        "CreatePrim",
                        prim_path=floor_path,
                        prim_type="Plane",
                        attributes={"size": 100.0}  # Large ground plane
                    )
                    
                    # Add physics properties to the ground
                    # omni.kit.commands.execute(
                    #     "CreatePhysics",
                    #     prim_path=floor_path,
                    #     physics_type="collider",
                    #     attributes={
                    #         "static": True,
                    #         "collision_enabled": True
                    #     }
                    # )
                # objects = [
                # {"path": "/World/Cube", "type": "Cube", "size": 20, "position": (0, 100, 0), "rotation": [1, 2, 3, 0], "scale": [1, 1, 1], "color": [0.5, 0.5, 0.5, 1.0], "physics_enabled": True, "mass": 1.0, "is_kinematic": False},
                # {"path": "/World/Sphere", "type": "Sphere", "radius": 5, "position": (5, 200, 0)},
                # {"path": "/World/Cone", "type": "Cone", "height": 8, "radius": 3, "position": (-5, 150, 0)}
                # ]
                print("start create objects: ", objects)
                objects_created = 0
                # Create each object
                for i, obj in enumerate(objects):
                    obj_name = obj.get("name", f"object_{i}")
                    obj_type = obj.get("type", "Cube")
                    obj_position = obj.get("position", [0, 0, 0])
                    obj_rotation = obj.get("rotation", [1, 0, 0, 0])  # Default is no rotation (identity quaternion)
                    obj_scale = obj.get("scale", [1, 1, 1])
                    obj_color = obj.get("color", [0.5, 0.5, 0.5, 1.0])
                    obj_physics = obj.get("physics_enabled", True)
                    obj_mass = obj.get("mass", 1.0)
                    obj_kinematic = obj.get("is_kinematic", False)
                    
                    # Create the object
                    obj_path = obj.get("path", f"/World/{obj_name}")
                    print("obj_path: ", obj_path)
                    if stage.GetPrimAtPath(obj_path):
                        print("obj_path already exists and skip creating")
                        continue
                    
                    # Create the primitive based on type
                    if obj_type in ["Cube", "Sphere", "Cylinder", "Cone", "Plane"]:
                        omni.kit.commands.execute(
                            "CreatePrim",
                            prim_path=obj_path,
                            prim_type=obj_type,
                            attributes={
                                "size": obj.get("size", 100.0), 
                                "position": obj_position, 
                                "rotation": obj_rotation, 
                                "scale": obj_scale, 
                                "color": obj_color, 
                                "physics_enabled": obj_physics,
                                "mass": obj_mass,
                                "is_kinematic": obj_kinematic} if obj_type in ["Cube", "Sphere","Plane"] else {},
                        )
                        print(f"Created {obj_type} at {obj_path}")
                    else:
                        return {"status": "error", "message": f"Invalid object type: {obj_type}"}
                    
                    # Set the transform
                    omni.kit.commands.execute(
                        "TransformPrimSRT",
                        path=obj_path,
                        new_translation=obj_position,
                        new_rotation_euler=[0, 0, 0],  # We'll set the quaternion separately
                        new_scale=obj_scale,
                    )
                    print(f"Created TransformPrimSRT at {obj_position}")
                    # Set rotation as quaternion
                    xform = UsdGeom.Xformable(stage.GetPrimAtPath(obj_path))
                    if xform and obj_rotation != [1, 0, 0, 0]:
                        quat = Gf.Quatf(obj_rotation[0], obj_rotation[1], obj_rotation[2], obj_rotation[3])
                        xform_op = xform.AddRotateOp()
                        xform_op.Set(quat)
                    
                    # Add physics properties if enabled
                    if obj_physics:
                        omni.kit.commands.execute(
                            "CreatePhysics",
                            prim_path=obj_path,
                            physics_type="rigid_body" if not obj_kinematic else "kinematic_body",
                            attributes={
                                "mass": obj_mass,
                                "collision_enabled": True,
                                "kinematic": obj_kinematic
                            }
                        )
                    print(f"Created Physics at {obj_path}")
                    # Set the color
                    if obj_color:
                        material_path = f"{obj_path}/material"
                        omni.kit.commands.execute(
                            "CreatePrim",
                            prim_path=material_path,
                            prim_type="Material",
                            attributes={
                                "diffuseColor": obj_color[:3],
                                "opacity": obj_color[3] if len(obj_color) > 3 else 1.0
                            }
                        )
                        print(f"Created Material at {material_path}")
                        # Bind the material to the object
                        omni.kit.commands.execute(
                            "BindMaterial",
                            material_path=material_path,
                            prim_path=obj_path
                        )

                        print(f"Bound Material to {obj_path}")
                        # increment the number of objects created
                        objects_created += 1
                return {
                    "status": "success",
                    "message": f"Created physics scene with {objects_created} objects",
                    "result": scene_name
                }
                
            except Exception as e:
                import traceback
                return {
                    "status": "error",
                    "message": str(e),
                    "traceback": traceback.format_exc()
                }
   
    def generate_3d_from_text_or_image(self, text_prompt=None, image_url=None, position=(0, 0, 50), scale=(10, 10, 10)):
        """
        Generate a 3D model from text or image, load it into the scene and transform it.
        
        Args:
            text_prompt (str, optional): Text prompt for 3D generation
            image_url (str, optional): URL of image for 3D generation
            position (tuple, optional): Position to place the model
            scale (tuple, optional): Scale of the model
            
        Returns:
            dict: Dictionary with the task_id and prim_path
        """
        try:
            # Initialize Beaver3d
            beaver = Beaver3d()
            
            # Determine generation method based on inputs
            # if image_url and text_prompt:
            #     # Generate 3D from image with text prompt as options
            #     task_id = beaver.generate_3d_from_image(image_url, text_prompt)
            #     print(f"3D model generation from image with text options started with task ID: {task_id}")
            # Check if we have cached task IDs for this input
            if not hasattr(self, '_image_url_cache'):
                self._image_url_cache = {}  # Cache for image URL to task_id mapping
            
            if not hasattr(self, '_text_prompt_cache'):
                self._text_prompt_cache = {}  # Cache for text prompt to task_id mapping
            
            # Check if we can retrieve task_id from cache
            task_id = None
            if image_url and image_url in self._image_url_cache:
                task_id = self._image_url_cache[image_url]
                print(f"Using cached task ID: {task_id} for image URL: {image_url}")
            elif text_prompt and text_prompt in self._text_prompt_cache:
                task_id = self._text_prompt_cache[text_prompt]
                print(f"Using cached task ID: {task_id} for text prompt: {text_prompt}")

            if task_id: #cache hit
                print(f"Using cached model ID: {task_id}")
            elif image_url:
                # Generate 3D from image only
                task_id = beaver.generate_3d_from_image(image_url)
                print(f"3D model generation from image started with task ID: {task_id}")
            elif text_prompt:
                # Generate 3D from text
                task_id = beaver.generate_3d_from_text(text_prompt)
                print(f"3D model generation from text started with task ID: {task_id}")
            else:
                return {
                    "status": "error",
                    "message": "Either text_prompt or image_url must be provided"
                }
            
            # Monitor the task and download the result
            # result_path = beaver.monitor_task_status(task_id)
            # task = asyncio.create_task(
                # beaver.monitor_task_status_async(
                    # task_id, on_complete_callback=load_model_into_scene))
            #await task
            def load_model_into_scene(task_id, status, result_path):
                print(f"{task_id} is {status}, 3D model  downloaded to: {result_path}")
                # Only cache the task_id after successful download
                if image_url and image_url not in self._image_url_cache:
                    self._image_url_cache[image_url] = task_id
                elif text_prompt and text_prompt not in self._text_prompt_cache:
                    self._text_prompt_cache[text_prompt] = task_id
                # Load the model into the scene
                loader = USDLoader()
                prim_path = loader.load_usd_model(task_id=task_id)
                
                # Load texture and create material
                try:
                    texture_path, material = loader.load_texture_and_create_material(task_id=task_id)
                    
                    # Bind texture to model
                    loader.bind_texture_to_model()
                except Exception as e:
                    print(f"Warning: Texture loading failed, continuing without texture: {str(e)}")
                
                # Transform the model
                loader.transform(position=position, scale=scale)
            
                return {
                    "status": "success",
                    "task_id": task_id,
                    "prim_path": prim_path
                }
            
            from omni.kit.async_engine import run_coroutine
            task = run_coroutine(beaver.monitor_task_status_async(
                task_id, on_complete_callback=load_model_into_scene))
            
            return {
                    "status": "success",
                    "task_id": task_id,
                    "message": f"3D model generation started with task ID: {task_id}"
            }
            
            
            
        except Exception as e:
            print(f"Error generating 3D model: {str(e)}")
            traceback.print_exc()
            return {
                "status": "error",
                "message": str(e)
            }
    
    def search_3d_usd_by_text(self, text_prompt:str, target_path:str, position=(0, 0, 50), scale=(10, 10, 10)):
        """
        Search a USD assets in USD Search service, load it into the scene and transform it.
        
        Args:
            text_prompt (str, optional): Text prompt for 3D generation
            target_path (str, ): target path in current scene stage
            position (tuple, optional): Position to place the model
            scale (tuple, optional): Scale of the model
            
        Returns:
            dict: Dictionary with prim_path
        """
        try:
            if text_prompt:
                print(f"3D model generation from text: {text_prompt}")
            else:
                return {
                    "status": "error",
                    "message": "text_prompt must be provided"
                }
            
            searcher3d = USDSearch3d()
            url = searcher3d.search( text_prompt )

            loader = USDLoader()
            prim_path = loader.load_usd_from_url( url, target_path )
            print(f"loaded url {url} to scene, prim path is: {prim_path}")
            # TODO: transform the model, need to fix the transform function for loaded USD
            # loader.transform(prim=prim_path, position=position, scale=scale)
            
            return {
                    "status": "success",
                    "prim_path": prim_path,
                    "message": f"3D model searching with prompt: {text_prompt}, return url: {url}, prim path in current scene: {prim_path}"
            }
        except Exception as e:
            print(f"Error searching 3D model: {str(e)}")
            traceback.print_exc()
            return {
                "status": "error",
                "message": str(e)
            }
    
    def transform(self, prim_path, position=(0, 0, 50), scale=(10, 10, 10)):
        """
        Transform a USD model by applying position and scale.
        
        Args:
            prim_path (str): Path to the USD prim to transform
            position (tuple, optional): The position to set (x, y, z)
            scale (tuple, optional): The scale to set (x, y, z)
            
        Returns:
            dict: Result information
        """
        try:
            # Get the USD context
            stage = omni.usd.get_context().get_stage()
            
            # Get the prim
            prim = stage.GetPrimAtPath(prim_path)
            if not prim:
                return {
                    "status": "error",
                    "message": f"Prim not found at path: {prim_path}"
                }
            
            # Initialize USDLoader
            loader = USDLoader()
            
            # Transform the model
            xformable = loader.transform(prim=prim, position=position, scale=scale)
            
            return {
                "status": "success",
                "message": f"Model at {prim_path} transformed successfully",
                "position": position,
                "scale": scale
            }
        except Exception as e:
            print(f"Error transforming model: {str(e)}")
            traceback.print_exc()
            return {
                "status": "error",
                "message": str(e)
            }

    def list_prims(self) -> Dict[str, Any]:
        """List all prim paths in the current USD scene."""
        try:
            import omni.usd
            
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return {
                    "status": "error",
                    "message": "No active stage found"
                }
            
            # Collect all prims
            prims_list = []
            for prim in stage.Traverse():
                prim_info = {
                    "path": str(prim.GetPath()),
                    "name": prim.GetName(),
                    "type": prim.GetTypeName()
                }
                prims_list.append(prim_info)
            
            return {
                "status": "success",
                "message": f"Found {len(prims_list)} prims in scene",
                "prims": prims_list,
                "total_count": len(prims_list)
            }
            
        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to list prims: {str(e)}"
            }

    def open_usd(self, usd_path: str) -> Dict[str, Any]:
        """Open a USD file as the main stage in Isaac Sim."""
        try:
            from omni.isaac.core.utils.stage import open_stage
            
            # Open the USD file as the main stage
            open_stage(usd_path=usd_path)
            print(f"Opened USD stage: {usd_path}")
            
            return {
                "status": "success",
                "message": f"Successfully opened USD stage: {usd_path}",
                "usd_path": usd_path
            }
            
        except Exception as e:
            import traceback
            return {
                "status": "error",
                "message": f"Failed to open USD stage: {str(e)}",
                "usd_path": usd_path,
                "traceback": traceback.format_exc()
            }

    def import_usd(self, usd_path: str, prim_path: str = None, position: List[float] = None, orientation: List[float] = None, orientation_format: str = "degrees") -> Dict[str, Any]:
        """Import a USD file as a prim into the Isaac Sim stage with flexible orientation input."""
        try:
            import os
            from omni.isaac.core.utils.stage import add_reference_to_stage
            from pxr import Usd, UsdGeom, Gf
            import numpy as np
            from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
            import omni.usd
            import time
            
            # Auto-generate prim path from filename if not provided
            if prim_path is None:
                filename = os.path.splitext(os.path.basename(usd_path))[0]
                prim_path = f"/World/{filename}"
                
                # Make sure prim path is unique
                stage = omni.usd.get_context().get_stage()
                counter = 1
                original_path = prim_path
                while stage.GetPrimAtPath(prim_path).IsValid():
                    prim_path = f"{original_path}_{counter}"
                    counter += 1
            
            # Import USD file first
            print(f"Importing USD from {usd_path} to {prim_path}")
            add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            
            # Wait for the prim to be fully created
            max_attempts = 10
            attempt = 0
            stage = omni.usd.get_context().get_stage()
            prim = None
            
            while attempt < max_attempts:
                prim = stage.GetPrimAtPath(prim_path)
                if prim.IsValid():
                    print(f"Prim found at {prim_path} after {attempt} attempts")
                    break
                print(f"Waiting for prim creation, attempt {attempt + 1}")
                time.sleep(0.1)
                attempt += 1
            
            if not prim or not prim.IsValid():
                return {
                    "status": "error",
                    "message": f"Prim at {prim_path} not found after import. Import may have failed."
                }
            
            # Apply default values if not provided
            if position is None:
                position = [0.0, 0.0, 0.0]
            
            if orientation is None:
                orientation = [0.0, 0.0, 0.0]
                orientation_format = "degrees"
            
            # Make sure prim is transformable
            if not prim.IsA(UsdGeom.Xformable):
                xform = UsdGeom.Xform.Define(stage, prim_path)
                print(f"Converted prim to Xformable")
            else:
                xform = UsdGeom.Xform(prim)
            
            # CRITICAL: Detect existing orient operation precision BEFORE clearing (same as move_prim)
            existing_orient_precision = None
            existing_ops = xform.GetOrderedXformOps()
            for op in existing_ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    # Get the precision from existing operation
                    if "quatd" in str(op.GetAttr().GetTypeName()):
                        existing_orient_precision = UsdGeom.XformOp.PrecisionDouble
                        print("Detected existing double precision quaternion")
                    elif "quatf" in str(op.GetAttr().GetTypeName()):
                        existing_orient_precision = UsdGeom.XformOp.PrecisionFloat
                        print("Detected existing float precision quaternion")
                    break
            
            # Clear and recreate with matching precision (same as move_prim)
            xform.ClearXformOpOrder()
            
            # Apply translation (always double precision for position)
            position_vec = Gf.Vec3d(position[0], position[1], position[2])
            xform.AddTranslateOp().Set(position_vec)
            
            # Handle orientation conversion (same logic as move_prim)
            if orientation_format.lower() == "quaternion" and len(orientation) == 4:
                final_quaternion = orientation
                print(f"Using quaternion orientation: {orientation}")
            elif orientation_format.lower() == "radians" and len(orientation) == 3:
                rpy_rad = np.array(orientation)
                quat_xyzw = euler_angles_to_quats(rpy_rad)
                final_quaternion = [quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]]  # w,x,y,z
                print(f"Converted radian orientation {orientation} to quaternion {final_quaternion}")
            else:
                # Default: Input is RPY in degrees
                rpy_deg = np.array(orientation)
                rpy_rad = np.deg2rad(rpy_deg)
                quat_xyzw = euler_angles_to_quats(rpy_rad)
                final_quaternion = [quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]]  # w,x,y,z
                print(f"Converted degree orientation {orientation} to quaternion {final_quaternion}")
            
            # Apply rotation with MATCHING precision (this is the key fix!)
            if existing_orient_precision == UsdGeom.XformOp.PrecisionFloat:
                quat = Gf.Quatf(final_quaternion[0], final_quaternion[1], final_quaternion[2], final_quaternion[3])
                orient_op = xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
                print("Using float precision for quaternion")
            else:
                quat = Gf.Quatd(final_quaternion[0], final_quaternion[1], final_quaternion[2], final_quaternion[3])
                orient_op = xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
                print("Using double precision for quaternion")
            
            orient_op.Set(quat)
            
            print(f"Successfully imported and positioned {prim_path}")
            print(f"Final position: {position}")
            print(f"Final orientation: {orientation} ({orientation_format})")
            
            # Verify the transform was applied by reading it back
            try:
                translate_attr = prim.GetAttribute("xformOp:translate")
                orient_attr = prim.GetAttribute("xformOp:orient")
                
                actual_position = [0, 0, 0]
                actual_quaternion = [1, 0, 0, 0]
                
                if translate_attr.IsValid():
                    translate_value = translate_attr.Get()
                    if translate_value:
                        actual_position = [float(translate_value[0]), float(translate_value[1]), float(translate_value[2])]
                
                if orient_attr.IsValid():
                    orient_value = orient_attr.Get()
                    if orient_value:
                        actual_quaternion = [float(orient_value.GetReal()), float(orient_value.GetImaginary()[0]), 
                                        float(orient_value.GetImaginary()[1]), float(orient_value.GetImaginary()[2])]
                
                print(f"Verified - Actual position: {actual_position}")
                print(f"Verified - Actual quaternion: {actual_quaternion}")
                
            except Exception as e:
                print(f"Could not verify transform: {str(e)}")
            
            return {
                "status": "success",
                "message": f"Successfully imported USD file at {prim_path} with position {position} and orientation {orientation} ({orientation_format})",
                "usd_path": usd_path,
                "prim_path": prim_path,
                "final_position": position,
                "final_orientation": orientation,
                "orientation_format": orientation_format
            }
            
        except Exception as e:
            import traceback
            print(f"Error in import_usd: {str(e)}")
            traceback.print_exc()
            return {
                "status": "error",
                "message": f"Failed to import USD file: {str(e)}",
                "usd_path": usd_path,
                "prim_path": prim_path,
                "traceback": traceback.format_exc()
            }

    def get_object_info(self, prim_path: str) -> Dict[str, Any]:
        """Get comprehensive object information including pose using direct USD attribute access."""
        try:
            import omni.usd
            from pxr import UsdGeom, Gf
            import numpy as np
            from omni.isaac.core.utils.numpy.rotations import quats_to_euler_angles
            
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            
            if not prim.IsValid():
                return {
                    "status": "error",
                    "message": f"Prim at {prim_path} not found"
                }
            
            # Basic prim info
            prim_info = {
                "path": str(prim.GetPath()),
                "name": prim.GetName(),
                "type": prim.GetTypeName(),
                "is_valid": prim.IsValid(),
                "is_active": prim.IsActive(),
                "has_children": bool(prim.GetChildren())
            }
            
            # Direct attribute access using your code
            translate_attr = prim.GetAttribute("xformOp:translate")
            orient_attr = prim.GetAttribute("xformOp:orient")
            scale_attr = prim.GetAttribute("xformOp:scale")
            
            # Get values
            position = [0.0, 0.0, 0.0]
            quaternion = [1.0, 0.0, 0.0, 0.0]  # Identity quaternion
            scale = [1.0, 1.0, 1.0]
            
            if translate_attr.IsValid():
                translate_value = translate_attr.Get()
                if translate_value:
                    position = [float(translate_value[0]), float(translate_value[1]), float(translate_value[2])]
            
            if orient_attr.IsValid():
                orient_value = orient_attr.Get()
                if orient_value:
                    quaternion = [float(orient_value.GetReal()), float(orient_value.GetImaginary()[0]), 
                                float(orient_value.GetImaginary()[1]), float(orient_value.GetImaginary()[2])]
            
            if scale_attr.IsValid():
                scale_value = scale_attr.Get()
                if scale_value:
                    scale = [float(scale_value[0]), float(scale_value[1]), float(scale_value[2])]
            
            # Convert quaternion to euler angles using your code
            if abs(quaternion[0] - 1.0) < 1e-6 and abs(quaternion[1]) < 1e-6 and abs(quaternion[2]) < 1e-6 and abs(quaternion[3]) < 1e-6:
                # Identity quaternion - no rotation
                rotation_radians = [0.0, 0.0, 0.0]
                rotation_degrees = [0.0, 0.0, 0.0]
            else:
                # Convert w,x,y,z to x,y,z,w format for Isaac function
                quat_xyzw = np.array([quaternion[0], quaternion[1], quaternion[2], quaternion[3]])
                rpy_rad = quats_to_euler_angles(quat_xyzw.reshape(1, 4))[0]
                rpy_deg = np.rad2deg(rpy_rad)
                rotation_radians = [float(rpy_rad[0]), float(rpy_rad[1]), float(rpy_rad[2])]
                rotation_degrees = [float(rpy_deg[0]), float(rpy_deg[1]), float(rpy_deg[2])]
            
            return {
                "status": "success",
                "message": f"Successfully retrieved info for {prim_path}",
                "prim_info": prim_info,
                "position": position,
                "rotation_quaternion": quaternion,  # [w, x, y, z]
                "rotation_radians": rotation_radians,  # [roll, pitch, yaw]
                "rotation_degrees": rotation_degrees,  # [roll, pitch, yaw]
                "scale": scale
            }
            
        except Exception as e:
            import traceback
            return {
                "status": "error",
                "message": f"Failed to get object info: {str(e)}",
                "traceback": traceback.format_exc()
            }

    def move_prim(self, prim_path: str, position: List[float] = None, orientation: List[float] = None, 
                orientation_format: str = "degrees") -> Dict[str, Any]:
        """Move a prim to a new pose with flexible orientation input (degrees/radians/quaternions)."""
        try:
            import omni.usd
            from pxr import UsdGeom, Gf
            import numpy as np
            from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
            
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            
            if not prim.IsValid():
                return {
                    "status": "error",
                    "message": f"Prim at {prim_path} not found"
                }
            
            # Read current pose using direct attribute access (same as your working code)
            translate_attr = prim.GetAttribute("xformOp:translate")
            orient_attr = prim.GetAttribute("xformOp:orient")
            
            # Get current values as defaults
            current_position = [0.0, 0.0, 0.0]
            current_quaternion = [1.0, 0.0, 0.0, 0.0]
            
            if translate_attr.IsValid():
                translate_value = translate_attr.Get()
                if translate_value:
                    current_position = [float(translate_value[0]), float(translate_value[1]), float(translate_value[2])]
            
            if orient_attr.IsValid():
                orient_value = orient_attr.Get()
                if orient_value:
                    current_quaternion = [float(orient_value.GetReal()), float(orient_value.GetImaginary()[0]), 
                                        float(orient_value.GetImaginary()[1]), float(orient_value.GetImaginary()[2])]
            
            # Use provided values or keep current ones
            final_position = position if position is not None else current_position
            
            # Handle orientation conversion (same logic as working script)
            if orientation is not None:
                if orientation_format.lower() == "quaternion" and len(orientation) == 4:
                    # Input is quaternion [w, x, y, z]
                    final_quaternion = orientation
                    print(f"Using quaternion orientation: {orientation}")
                elif orientation_format.lower() == "radians" and len(orientation) == 3:
                    # Input is RPY in radians
                    rpy_rad = np.array(orientation)
                    quat_xyzw = euler_angles_to_quats(rpy_rad)
                    final_quaternion = [quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]]  # w,x,y,z
                    print(f"Converted radian orientation {orientation} to quaternion {final_quaternion}")
                else:
                    # Default: Input is RPY in degrees
                    rpy_deg = np.array(orientation)
                    rpy_rad = np.deg2rad(rpy_deg)
                    quat_xyzw = euler_angles_to_quats(rpy_rad)
                    final_quaternion = [quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]]  # w,x,y,z
                    print(f"Converted degree orientation {orientation} to quaternion {final_quaternion}")
            else:
                # Keep current orientation
                final_quaternion = current_quaternion
                orientation = "unchanged"
            
            # Apply transformations (detect existing precision and match it - same as working script)
            if not prim.IsA(UsdGeom.Xformable):
                return {
                    "status": "error",
                    "message": f"Prim {prim_path} is not transformable"
                }
            
            xform = UsdGeom.Xform(prim)
            
            # Check existing orient operation precision BEFORE clearing
            existing_orient_precision = None
            existing_ops = xform.GetOrderedXformOps()
            for op in existing_ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    # Get the precision from existing operation
                    if "quatd" in str(op.GetAttr().GetTypeName()):
                        existing_orient_precision = UsdGeom.XformOp.PrecisionDouble
                    elif "quatf" in str(op.GetAttr().GetTypeName()):
                        existing_orient_precision = UsdGeom.XformOp.PrecisionFloat
                    break
            
            # Clear and recreate with matching precision
            xform.ClearXformOpOrder()
            
            # Apply translation (always double precision for position)
            position_vec = Gf.Vec3d(final_position[0], final_position[1], final_position[2])
            xform.AddTranslateOp().Set(position_vec)
            
            # Apply rotation with matching precision
            if existing_orient_precision == UsdGeom.XformOp.PrecisionFloat:
                quat = Gf.Quatf(final_quaternion[0], final_quaternion[1], final_quaternion[2], final_quaternion[3])
                orient_op = xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
                print("Using float precision for quaternion")
            else:
                quat = Gf.Quatd(final_quaternion[0], final_quaternion[1], final_quaternion[2], final_quaternion[3])
                orient_op = xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
                print("Using double precision for quaternion")
            
            orient_op.Set(quat)
            
            print(f"Moved {prim_path} to position {final_position}, orientation {orientation}")
            
            return {
                "status": "success",
                "message": f"Successfully moved {prim_path} to new pose",
                "prim_path": prim_path,
                "previous_position": current_position,
                "previous_quaternion": current_quaternion,
                "new_position": final_position,
                "new_quaternion": final_quaternion,
                "orientation_format": orientation_format
            }
            
        except Exception as e:
            import traceback
            return {
                "status": "error",
                "message": f"Failed to move prim: {str(e)}",
                "prim_path": prim_path,
                "traceback": traceback.format_exc()
            }
