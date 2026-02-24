"""
Template Manager Module

Handles saving and loading of workpiece templates including:
- Point cloud data
- Hole position annotations
- Metadata
"""

import json
import pickle
from pathlib import Path
from typing import List, Dict, Optional
from dataclasses import dataclass, asdict
import numpy as np
import open3d as o3d


@dataclass
class HoleAnnotation:
    """Represents a single screw hole annotation."""
    id: int
    position: List[float]  # [x, y, z] in meters
    normal: Optional[List[float]] = None  # Surface normal [nx, ny, nz]
    hole_type: str = "standard"  # corner, edge, standard, deep_hole


@dataclass
class WorkpieceTemplate:
    """Complete workpiece template."""
    template_id: str
    timestamp: str
    pointcloud_path: str
    num_holes: int
    holes: List[Dict]  # List of hole annotations
    metadata: Dict

    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'WorkpieceTemplate':
        """Create from dictionary."""
        holes = [HoleAnnotation(**h) for h in data['holes']]
        data['holes'] = [asdict(h) for h in holes]
        return cls(**data)


class TemplateManager:
    """Manages workpiece templates storage and retrieval."""

    def __init__(self, base_path: str = "data/templates"):
        """
        Initialize template manager.

        Args:
            base_path: Base directory for template storage
        """
        self.base_path = Path(base_path)
        self.base_path.mkdir(parents=True, exist_ok=True)
        self.current_template: Optional[WorkpieceTemplate] = None

    def save_template(self,
                      template: WorkpieceTemplate,
                      pointcloud: o3d.geometry.PointCloud,
                      overwrite: bool = False) -> str:
        """
        Save template to disk.

        Args:
            template: WorkpieceTemplate object
            pointcloud: Open3D point cloud
            overwrite: Overwrite existing template

        Returns:
            Path to saved template file
        """
        # Create template directory
        template_dir = self.base_path / template.template_id
        template_dir.mkdir(exist_ok=True)

        # Check if exists
        json_path = template_dir / "template.json"
        if json_path.exists() and not overwrite:
            raise FileExistsError(f"Template {template.template_id} already exists")

        # Save point cloud
        pcd_path = template_dir / "pointcloud.pcd"
        o3d.io.write_point_cloud(str(pcd_path), pointcloud)

        # Update pointcloud path
        template.pointcloud_path = str(pcd_path)

        # Save JSON
        with open(json_path, 'w') as f:
            json.dump(template.to_dict(), f, indent=2)

        self.current_template = template

        return str(json_path)

    def load_template(self, template_id: str) -> WorkpieceTemplate:
        """
        Load template from disk.

        Args:
            template_id: Template identifier

        Returns:
            WorkpieceTemplate object
        """
        json_path = self.base_path / template_id / "template.json"

        if not json_path.exists():
            raise FileNotFoundError(f"Template {template_id} not found")

        with open(json_path, 'r') as f:
            data = json.load(f)

        self.current_template = WorkpieceTemplate.from_dict(data)

        return self.current_template

    def load_template_pointcloud(self, template_id: str) -> o3d.geometry.PointCloud:
        """
        Load point cloud for a template.

        Args:
            template_id: Template identifier

        Returns:
            Open3D point cloud
        """
        template = self.load_template(template_id)
        pcd_path = Path(template.pointcloud_path)

        if not pcd_path.exists():
            # Try relative path
            pcd_path = self.base_path / template_id / "pointcloud.pcd"

        return o3d.io.read_point_cloud(str(pcd_path))

    def list_templates(self) -> List[str]:
        """
        List all available templates.

        Returns:
            List of template IDs
        """
        templates = []
        for path in self.base_path.iterdir():
            if path.is_dir() and (path / "template.json").exists():
                templates.append(path.name)
        return sorted(templates)

    def delete_template(self, template_id: str) -> bool:
        """
        Delete a template.

        Args:
            template_id: Template identifier

        Returns:
            True if deleted, False if not found
        """
        template_dir = self.base_path / template_id

        if not template_dir.exists():
            return False

        # Remove all files
        for file in template_dir.iterdir():
            file.unlink()

        # Remove directory
        template_dir.rmdir()

        if self.current_template and self.current_template.template_id == template_id:
            self.current_template = None

        return True

    def create_empty_template(self,
                               template_id: str,
                               workpiece_name: str = "Unknown") -> WorkpieceTemplate:
        """
        Create a new empty template.

        Args:
            template_id: Template identifier
            workpiece_name: Human-readable name

        Returns:
            Empty WorkpieceTemplate
        """
        from datetime import datetime

        template = WorkpieceTemplate(
            template_id=template_id,
            timestamp=datetime.now().isoformat(),
            pointcloud_path="",  # Will be set when saving
            num_holes=0,
            holes=[],
            metadata={
                'workpiece_name': workpiece_name,
                'notes': ''
            }
        )

        return template

    def add_hole_annotation(self,
                            template: WorkpieceTemplate,
                            hole_id: int,
                            position: List[float],
                            normal: Optional[List[float]] = None,
                            hole_type: str = "standard") -> WorkpieceTemplate:
        """
        Add or update a hole annotation.

        Args:
            template: Template to modify
            hole_id: Hole identifier
            position: [x, y, z] position
            normal: Optional surface normal
            hole_type: Type of hole

        Returns:
            Updated template
        """
        # Check if hole already exists
        hole_dict = {
            'id': hole_id,
            'position': position,
            'normal': normal,
            'hole_type': hole_type
        }

        # Update or add
        existing_idx = None
        for i, h in enumerate(template.holes):
            if h['id'] == hole_id:
                existing_idx = i
                break

        if existing_idx is not None:
            template.holes[existing_idx] = hole_dict
        else:
            template.holes.append(hole_dict)

        template.num_holes = len(template.holes)

        return template

    def get_hole_positions(self, template: WorkpieceTemplate) -> np.ndarray:
        """
        Get all hole positions as numpy array.

        Args:
            template: WorkpieceTemplate

        Returns:
            Nx3 array of positions
        """
        positions = []
        for hole in template.holes:
            positions.append(hole['position'])
        return np.array(positions)


class AnnotationSession:
    """Manages an active annotation session."""

    def __init__(self, template_id: str, workpiece_name: str = "Unknown"):
        """
        Start a new annotation session.

        Args:
            template_id: Template identifier
            workpiece_name: Human-readable name
        """
        self.template_manager = TemplateManager()
        self.template = self.template_manager.create_empty_template(
            template_id, workpiece_name
        )
        self.pointcloud: Optional[o3d.geometry.PointCloud] = None
        self.next_hole_id = 1

    def set_pointcloud(self, pcd: o3d.geometry.PointCloud):
        """Set the point cloud for this session."""
        self.pointcloud = pcd

    def add_hole_at_position(self, position: List[float]) -> int:
        """
        Add a hole annotation at clicked position.

        Args:
            position: [x, y, z] position

        Returns:
            Assigned hole ID
        """
        hole_id = self.next_hole_id

        self.template = self.template_manager.add_hole_annotation(
            self.template,
            hole_id=hole_id,
            position=position,
            hole_type="standard"
        )

        self.next_hole_id += 1
        return hole_id

    def remove_hole(self, hole_id: int) -> bool:
        """
        Remove a hole annotation.

        Args:
            hole_id: Hole to remove

        Returns:
            True if removed
        """
        for i, hole in enumerate(self.template.holes):
            if hole['id'] == hole_id:
                self.template.holes.pop(i)
                self.template.num_holes = len(self.template.holes)
                return True
        return False

    def save(self) -> str:
        """
        Save the annotation session as a template.

        Returns:
            Path to saved template

        Raises:
            ValueError: If no point cloud set
        """
        if self.pointcloud is None:
            raise ValueError("No point cloud set")

        return self.template_manager.save_template(
            self.template,
            self.pointcloud,
            overwrite=False
        )

    def get_summary(self) -> Dict:
        """Get session summary."""
        return {
            'template_id': self.template.template_id,
            'num_holes': self.template.num_holes,
            'holes': self.template.holes,
            'pointcloud_loaded': self.pointcloud is not None
        }
