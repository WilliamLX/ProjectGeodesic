"""
Teaching GUI

PyQt6 application for teaching workpiece templates:
- Load and visualize point clouds
- Click to annotate screw hole positions
- Save templates
"""

import sys
import numpy as np
import open3d as o3d
from pathlib import Path

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QListWidget, QLineEdit, QTextEdit,
    QFileDialog, QMessageBox, QGroupBox, QFormLayout
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QSurfaceFormat

from .template_manager import AnnotationSession, TemplateManager
from .pointcloud_proc import ros_to_open3d


class VisualizerWidget(QWidget):
    """Open3D visualization widget embedded in PyQt."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pointcloud = None
        self.hole_markers = []
        self.selected_hole_idx = -1

    def set_pointcloud(self, pcd: o3d.geometry.PointCloud):
        """Set the point cloud to visualize."""
        self.pointcloud = pcd
        self.update()

    def add_hole_marker(self, position: np.ndarray, hole_id: int):
        """Add a visual marker for a hole."""
        # Create a sphere marker
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.003)
        sphere.translate(position)
        sphere.paint_uniform_color([1, 0, 0])  # Red
        self.hole_markers.append(sphere)
        self.update()

    def clear_hole_markers(self):
        """Clear all hole markers."""
        self.hole_markers = []
        self.update()

    def update(self):
        """Update visualization."""
        if self.pointcloud is None:
            return

        # Create visualization
        geometries = [self.pointcloud] + self.hole_markers

        # Note: Full Open3D embedding in PyQt is complex
        # For simplicity, we'll use a non-blocking visualizer
        pass


class TeachingGUI(QMainWindow):
    """Main teaching GUI window."""

    def __init__(self):
        super().__init__()

        self.session = None
        self.template_manager = TemplateManager()
        self.current_template_id = None

        self.init_ui()
        self.init_open3d()

    def init_ui(self):
        """Initialize user interface."""
        self.setWindowTitle('ProjectGeodesic - Teaching Mode')
        self.setGeometry(100, 100, 1400, 800)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QHBoxLayout(central_widget)

        # Left panel (controls)
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, 1)

        # Center panel (3D view placeholder)
        center_panel = self.create_3d_panel()
        main_layout.addWidget(center_panel, 3)

        # Right panel (hole list)
        right_panel = self.create_hole_list_panel()
        main_layout.addWidget(right_panel, 1)

    def create_control_panel(self) -> QGroupBox:
        """Create left control panel."""
        panel = QGroupBox("Controls")
        layout = QVBoxLayout()

        # Template section
        template_group = QGroupBox("Template")
        template_layout = QFormLayout()

        self.template_id_edit = QLineEdit()
        self.template_id_edit.setPlaceholderText("e.g., nio_front_roof_v1")
        template_layout.addRow("ID:", self.template_id_edit)

        self.workpiece_name_edit = QLineEdit()
        self.workpiece_name_edit.setPlaceholderText("e.g., NIO Front Roof")
        template_layout.addRow("Name:", self.workpiece_name_edit)

        layout.addWidget(template_group)

        # Point cloud section
        pcd_group = QGroupBox("Point Cloud")
        pcd_layout = QVBoxLayout()

        self.load_pcd_btn = QPushButton("Load Point Cloud")
        self.load_pcd_btn.clicked.connect(self.load_pointcloud)
        pcd_layout.addWidget(self.load_pcd_btn)

        self.pcd_info_label = QLabel("No point cloud loaded")
        self.pcd_info_label.setWordWrap(True)
        pcd_layout.addWidget(self.pcd_info_label)

        layout.addWidget(pcd_group)

        # Session section
        session_group = QGroupBox("Session")
        session_layout = QVBoxLayout()

        self.new_session_btn = QPushButton("New Annotation Session")
        self.new_session_btn.clicked.connect(self.new_session)
        session_layout.addWidget(self.new_session_btn)

        self.save_template_btn = QPushButton("Save Template")
        self.save_template_btn.clicked.connect(self.save_template)
        self.save_template_btn.setEnabled(False)
        session_layout.addWidget(self.save_template_btn)

        layout.addWidget(session_group)

        # View section
        view_group = QGroupBox("View Controls")
        view_layout = QVBoxLayout()

        self.reset_view_btn = QPushButton("Reset View")
        self.reset_view_btn.clicked.connect(self.reset_view)
        view_layout.addWidget(self.reset_view_btn)

        self.toggle_normals_btn = QPushButton("Toggle Normals")
        self.toggle_normals_btn.clicked.connect(self.toggle_normals)
        view_layout.addWidget(self.toggle_normals_btn)

        layout.addWidget(view_group)

        # Instructions
        info_group = QGroupBox("Instructions")
        info_layout = QVBoxLayout()
        info_text = QTextEdit()
        info_text.setReadOnly(True)
        info_text.setPlainText(
            "1. Load a point cloud\n"
            "2. Click 'New Annotation Session'\n"
            "3. In 3D view, Shift+Click to add hole annotations\n"
            "4. Ctrl+Click to remove holes\n"
            "5. Save template when done"
        )
        info_layout.addWidget(info_text)
        layout.addWidget(info_group)

        layout.addStretch()
        panel.setLayout(layout)
        return panel

    def create_3d_panel(self) -> QGroupBox:
        """Create center 3D visualization panel."""
        panel = QGroupBox("3D View")
        layout = QVBoxLayout()

        self.view_label = QLabel(
            "Open3D Visualizer\n(Non-embedded window will appear)"
        )
        self.view_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.view_label)

        panel.setLayout(layout)
        return panel

    def create_hole_list_panel(self) -> QGroupBox:
        """Create right hole list panel."""
        panel = QGroupBox("Hole Annotations")
        layout = QVBoxLayout()

        self.hole_list = QListWidget()
        self.hole_list.itemClicked.connect(self.on_hole_selected)
        layout.addWidget(self.hole_list)

        # Hole details
        details_group = QGroupBox("Selected Hole")
        details_layout = QFormLayout()

        self.hole_id_label = QLabel("-")
        details_layout.addRow("ID:", self.hole_id_label)

        self.hole_x_label = QLabel("-")
        details_layout.addRow("X:", self.hole_x_label)

        self.hole_y_label = QLabel("-")
        details_layout.addRow("Y:", self.hole_y_label)

        self.hole_z_label = QLabel("-")
        details_layout.addRow("Z:", self.hole_z_label)

        layout.addWidget(details_group)

        # Remove button
        self.remove_hole_btn = QPushButton("Remove Selected Hole")
        self.remove_hole_btn.clicked.connect(self.remove_selected_hole)
        self.remove_hole_btn.setEnabled(False)
        layout.addWidget(self.remove_hole_btn)

        # Count label
        self.hole_count_label = QLabel("Holes: 0 / 13")
        self.hole_count_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.hole_count_label)

        layout.addStretch()
        panel.setLayout(layout)
        return panel

    def init_open3d(self):
        """Initialize Open3D visualizer."""
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("Teaching Mode - 3D View", 1920, 1080)

        # Register key callbacks
        self.vis.register_key_callback(65, self.on_shift_click)  # A key
        self.vis.register_key_callback(32, self.on_space_click)  # Space

        self.pointcloud = None
        self.hole_spheres = []

    def load_pointcloud(self):
        """Load point cloud from file."""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Load Point Cloud",
            str(Path.home()),
            "Point Cloud Files (*.pcd *.ply);;All Files (*)"
        )

        if file_path:
            try:
                self.pointcloud = o3d.io.read_point_cloud(file_path)

                # Update info
                info = f"Points: {len(self.pointcloud.points)}\n"
                if self.pointcloud.has_normals():
                    info += "Has normals: Yes"
                else:
                    info += "Has normals: No"

                self.pcd_info_label.setText(info)

                # Update visualizer
                self.vis.clear_geometries()
                self.vis.add_geometry(self.pointcloud, reset_bounding_box=True)

                # Estimate normals if needed
                if not self.pointcloud.has_normals():
                    self.pointcloud.estimate_normals()

                self.get_logger().info(f'Loaded point cloud: {file_path}')

            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load point cloud:\n{e}")

    def new_session(self):
        """Start a new annotation session."""
        if self.pointcloud is None:
            QMessageBox.warning(self, "Warning", "Please load a point cloud first")
            return

        template_id = self.template_id_edit.text().strip()
        if not template_id:
            QMessageBox.warning(self, "Warning", "Please enter a template ID")
            return

        workpiece_name = self.workpiece_name_edit.text().strip() or "Unknown"

        self.session = AnnotationSession(template_id, workpiece_name)
        self.session.set_pointcloud(self.pointcloud)

        self.save_template_btn.setEnabled(True)
        self.update_hole_list()

        self.get_logger().info(f'Started annotation session: {template_id}')

    def save_template(self):
        """Save the annotation session as a template."""
        if self.session is None:
            QMessageBox.warning(self, "Warning", "No active session")
            return

        try:
            path = self.session.save()
            QMessageBox.information(
                self,
                "Success",
                f"Template saved:\n{path}\n\nHoles: {self.session.template.num_holes}"
            )
            self.get_logger().info(f'Saved template: {path}')

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save template:\n{e}")

    def update_hole_list(self):
        """Update hole list widget."""
        if self.session is None:
            return

        self.hole_list.clear()
        for hole in self.session.template.holes:
            pos = hole['position']
            item_text = f"#{hole['id']}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
            self.hole_list.addItem(item_text)

        self.hole_count_label.setText(
            f"Holes: {self.session.template.num_holes} / 13"
        )

    def on_hole_selected(self, item):
        """Handle hole selection in list."""
        row = self.hole_list.row(item)
        if self.session is None or row >= len(self.session.template.holes):
            return

        hole = self.session.template.holes[row]
        pos = hole['position']

        self.hole_id_label.setText(str(hole['id']))
        self.hole_x_label.setText(f"{pos[0]:.4f}")
        self.hole_y_label.setText(f"{pos[1]:.4f}")
        self.hole_z_label.setText(f"{pos[2]:.4f}")

        self.remove_hole_btn.setEnabled(True)

        # Highlight in visualizer
        self.highlight_hole(pos)

    def remove_selected_hole(self):
        """Remove selected hole."""
        if self.session is None:
            return

        row = self.hole_list.currentRow()
        if row < 0:
            return

        hole = self.session.template.holes[row]
        hole_id = hole['id']

        if self.session.remove_hole(hole_id):
            self.update_hole_list()
            self.remove_hole_btn.setEnabled(False)
            self.update_visualizer_markers()

    def highlight_hole(self, position):
        """Highlight a hole in the visualizer."""
        # Remove old highlight
        self.vis.remove_geometry(self.highlight_sphere, reset_bounding_box=False)

        # Create new highlight
        self.highlight_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
        self.highlight_sphere.translate(position)
        self.highlight_sphere.paint_uniform_color([1, 1, 0])  # Yellow
        self.vis.add_geometry(self.highlight_sphere, reset_bounding_box=False)
        self.vis.poll()
        self.vis.update_renderer()

    def update_visualizer_markers(self):
        """Update all hole markers in visualizer."""
        # Remove old spheres
        for sphere in self.hole_spheres:
            self.vis.remove_geometry(sphere, reset_bounding_box=False)
        self.hole_spheres.clear()

        # Add new spheres
        if self.session is not None:
            for hole in self.session.template.holes:
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.003)
                pos = hole['position']
                sphere.translate(pos)
                sphere.paint_uniform_color([1, 0, 0])  # Red
                self.vis.add_geometry(sphere, reset_bounding_box=False)
                self.hole_spheres.append(sphere)

        self.vis.poll()
        self.vis.update_renderer()

    def reset_view(self):
        """Reset 3D view."""
        self.vis.reset_view_point(True)
        self.vis.poll()
        self.vis.update_renderer()

    def toggle_normals(self):
        """Toggle normal visualization."""
        if self.pointcloud is None:
            return

        # Re-add with normals toggle
        # (This would require more complex state management)
        pass

    def on_shift_click(self, vis):
        """Handle Shift+Click for adding holes."""
        # This is a placeholder - actual implementation would
        # use ray casting from mouse position
        pass

    def on_space_click(self, vis):
        """Handle Space key for adding hole at center."""
        # Placeholder
        pass

    def get_logger(self):
        """Get logger instance."""
        import rclpy
        if not rclpy.ok():
            rclpy.init()
        return rclpy.logging.get_logger("teaching_gui")

    def closeEvent(self, event):
        """Handle window close."""
        if self.vis:
            self.vis.destroy_window()
        event.accept()


def main(args=None):
    """Main entry point."""
    app = QApplication(sys.argv)
    window = TeachingGUI()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
