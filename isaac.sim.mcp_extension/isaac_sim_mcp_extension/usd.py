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

import os
import omni.usd
from pathlib import Path
from pxr import UsdShade, Sdf, UsdGeom, Gf
from omni.isaac.core.utils.stage import add_reference_to_stage


class USDLoader:
    """
    A class to load USD models and textures from local directories,
    with methods to bind materials and transform models.
    """

    def __init__(self):
        """
        Initialize the USDLoader class with default values.
        """
        self.usd_prim = None
        self.material = None
        self.working_dir = Path(os.environ.get("USD_WORKING_DIR", "/tmp/usd"))
        self.stage = omni.usd.get_context().get_stage()

    def load_usd_model(self, abs_path=None, task_id=None):
        """
        Load a USD model from either an absolute path or by task_id.
        
        Args:
            abs_path (str, optional): Absolute path to the USD file
            task_id (str, optional): Task ID to load model from working_dir
            
        Returns:
            str: Path to the loaded USD prim
        """
        if not (abs_path or task_id):
            raise ValueError("Either abs_path or task_id must be provided")
            
        if task_id:
            usd_path = self.working_dir / task_id / "output.usd"
        else:
            usd_path = Path(abs_path)
            
        if not usd_path.exists():
            raise FileNotFoundError(f"USD file not found at: {usd_path}")
            
        # Create a unique prim path name based on task_id or file name
        if task_id:
            prim_id = task_id[-5:]  # Last 5 chars of task ID
        else:
            prim_id = usd_path.stem[:5]  # First 5 chars of filename
            
        usd_prim_path = f"/World/model_{prim_id}"
        
        # Add the USD to the stage
        self.usd_prim = add_reference_to_stage(str(usd_path), usd_prim_path)
        
        print(f"Loaded USD model from {usd_path} at {usd_prim_path}")
        return usd_prim_path
    
    def load_texture_and_create_material(self, abs_path=None, task_id=None):
        """
        Load a texture from either an absolute path or by task_id and create a material with it.
        
        Args:
            abs_path (str, optional): Absolute path to the texture file
            task_id (str, optional): Task ID to load texture from working_dir
            
        Returns:
            tuple: (str, UsdShade.Material) - Path to the loaded texture and the created material
        """
        if not (abs_path or task_id):
            raise ValueError("Either abs_path or task_id must be provided")
            
        # Load texture
        if task_id:
            texture_path = self.working_dir / task_id / "textures" / "material_0.png"
            material_id = task_id[-5:]  # Last 5 chars of task ID
        else:
            texture_path = Path(abs_path)
            material_id = texture_path.stem[:5]  # First 5 chars of filename
            
        if not texture_path.exists():
            raise FileNotFoundError(f"Texture file not found at: {texture_path}")
        
        # Create a unique material name
        material_path = f"/World/SimpleMaterial_{material_id}"
        
        # Create material
        material = UsdShade.Material.Define(self.stage, material_path)
        
        # Create shader
        shader = UsdShade.Shader.Define(self.stage, f"{material_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        
        # Connect shader to material
        material.CreateSurfaceOutput().ConnectToSource(shader.CreateOutput("surface", Sdf.ValueTypeNames.Token))
        
        # Create texture
        texture = UsdShade.Shader.Define(self.stage, f"{material_path}/Texture")
        texture.CreateIdAttr("UsdUVTexture")
        texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(texture_path))
        
        # Connect texture to shader's diffuse color
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        )
        
        print(f"Created material with texture at {material_path}")
        self.material = material
        return str(texture_path), material
    
    def bind_texture_to_model(self, prim=None, material=None):
        """
        Bind a texture to a USD model.
        
        Args:
            prim (UsdGeom.Xformable, optional): The USD prim to bind the material to.
                                               If None, uses self.usd_prim
            material (UsdShade.Material, optional): The material to bind.
                                                  If None, uses self.material
        
        Returns:
            bool: True if binding succeeded, False otherwise
        """
        if prim is None:
            prim = self.usd_prim
        
        if material is None:
            material = self.material
            
        if prim is None or material is None:
            raise ValueError("Both prim and material must be provided or previously set")
            
        try:
            binding_api = UsdShade.MaterialBindingAPI(prim)
            binding_api.Bind(material)
            print(f"Successfully bound material to {prim.GetPath()}")
            return True
        except Exception as e:
            print(f"Failed to bind material: {str(e)}")
            return False
    
    def transform(self, prim=None, position=(0, 0, 50), scale=(10, 10, 10)):
        """
        Transform a USD model by applying position and scale.
        
        Args:
            prim (UsdGeom.Xformable, optional): The USD prim to transform.
                                               If None, uses self.usd_prim
            position (tuple, optional): The position to set (x, y, z)
            scale (tuple, optional): The scale to set (x, y, z)
            
        Returns:
            UsdGeom.Xformable: The transformed prim
        """
        if prim is None:
            prim = self.usd_prim
            
        if prim is None:
            raise ValueError("Prim must be provided or previously set")
            
        # Get the Xformable interface
        xformable = UsdGeom.Xformable(prim)
        
        # Check if transform operations already exist and use them
        xform_ops = xformable.GetOrderedXformOps()
        
        # Handle translation
        translate_op = None
        for op in xform_ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        
        if translate_op:
            translate_op.Set(Gf.Vec3d(position))
        else:
            xformable.AddTranslateOp().Set(Gf.Vec3d(position))
        print(f"Model positioned at {position}")
        
        # Handle scaling
        scale_op = None
        for op in xform_ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                scale_op = op
                break
        
        if scale_op:
            scale_op.Set(Gf.Vec3d(scale))
        else:
            xformable.AddScaleOp().Set(Gf.Vec3d(scale))
        print(f"Model scaled to {scale}")
        
        return xformable
    
    @staticmethod
    def test_tasks_load():
        """
        Test method to load and process multiple task IDs.
        """
        loader = USDLoader()
        
        # List of task IDs to test
        task_ids = ["cgt-20250408202506-6tdc4", "cgt-20250408202622-rzcgg", "cgt-20250408202753-44b9f"]
        
        for task_id in task_ids:
            try:
                # Load model
                prim_path = loader.load_usd_model(task_id=task_id)
                
                # # Load texture
                # texture_path = loader.load_texture(task_id=task_id)
                
                # Create material with texture
                texture_path, material = loader.load_texture_and_create_material(task_id=task_id)
                
                # Bind texture to model
                loader.bind_texture_to_model()
                
                # Transform model
                loader.transform(position=(0, 0, 50 + task_ids.index(task_id) * 20))
                
                print(f"Successfully processed task {task_id}")
            except Exception as e:
                print(f"Error processing task {task_id}: {str(e)}")
    
    @staticmethod
    def test_absolute_paths():
        """
        Test method for loading from absolute paths.
        """
        loader = USDLoader()
        
        # Test with absolute paths
        model_path = "/tmp/usd/cgt-20250408202506-6tdc4/output.usd"
        texture_path = "/tmp/usd/cgt-20250408202506-6tdc4/textures/material_0.png"
        
        try:
            # Load model
            prim_path = loader.load_usd_model(abs_path=model_path)
            
            # Load texture
            texture_path, material = loader.load_texture_and_create_material(abs_path=texture_path)
            
            # Create material with texture
            # material = loader.create_material_with_texture(texture_path)

            # Bind texture to model
            loader.bind_texture_to_model()
            
            # Transform model
            loader.transform()
            
            print("Successfully tested absolute paths")
        except Exception as e:
            print(f"Error testing absolute paths: {str(e)}")


if __name__ == "__main__":
    # USDLoader.main()
    USDLoader.test_tasks_load()
    USDLoader.test_absolute_paths()
    
