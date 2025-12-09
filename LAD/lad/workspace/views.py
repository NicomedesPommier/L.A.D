from rest_framework import viewsets, status
from rest_framework.decorators import action, api_view, permission_classes
from rest_framework.response import Response
from rest_framework.permissions import IsAuthenticated
from django.shortcuts import get_object_or_404
import subprocess
import os
import json
import shlex

from .models import Canvas, WorkspaceFile, CustomMesh
from .serializers import CanvasSerializer, WorkspaceFileSerializer, FileTreeSerializer, CustomMeshSerializer
from django.core.files.base import ContentFile
import requests

# Docker container name for workspace operations
DOCKER_CONTAINER = "qcar_docker-ros-1"


class CanvasViewSet(viewsets.ModelViewSet):
    """
    API endpoint for managing canvases/workspaces
    """
    serializer_class = CanvasSerializer
    permission_classes = [IsAuthenticated]

    def get_queryset(self):
        return Canvas.objects.filter(user=self.request.user)

    def perform_create(self, serializer):
        serializer.save(user=self.request.user)

    @action(detail=True, methods=["get"])
    def file_tree(self, request, pk=None):
        """
        Get the file tree structure for this canvas
        Returns hierarchical structure similar to FileExplorer component
        """
        canvas = self.get_object()
        files = canvas.files.all()

        # Build tree structure
        tree = self._build_tree(files)

        serializer = FileTreeSerializer(tree, many=True)
        return Response(serializer.data)

    def _build_tree(self, files):
        """Build hierarchical tree from flat file list"""
        tree = {}

        for file in files:
            parts = file.path.split("/")
            current = tree

            for i, part in enumerate(parts):
                path = "/".join(parts[: i + 1])

                if part not in current:
                    is_last = i == len(parts) - 1
                    current[part] = {
                        "name": part,
                        "path": path,
                        "type": file.file_type if is_last else "directory",
                        "children": {} if not is_last else None,
                    }

                if i < len(parts) - 1:
                    current = current[part]["children"]

        # Convert dict to list
        def dict_to_list(node_dict):
            result = []
            for key, value in node_dict.items():
                item = {
                    "name": value["name"],
                    "path": value["path"],
                    "type": value["type"],
                }
                if value["children"] is not None:
                    item["children"] = dict_to_list(value["children"])
                result.append(item)
            return result

        return dict_to_list(tree)


class WorkspaceFileViewSet(viewsets.ModelViewSet):
    """
    API endpoint for file CRUD operations
    """
    serializer_class = WorkspaceFileSerializer
    permission_classes = [IsAuthenticated]

    def get_queryset(self):
        canvas_id = self.kwargs.get("canvas_pk")
        return WorkspaceFile.objects.filter(
            canvas_id=canvas_id, canvas__user=self.request.user
        )

    def perform_create(self, serializer):
        canvas_id = self.kwargs.get("canvas_pk")
        canvas = get_object_or_404(
            Canvas, id=canvas_id, user=self.request.user
        )
        file_instance = serializer.save(canvas=canvas)

        # Write to Docker volume
        self._write_to_docker(file_instance)

    def perform_update(self, serializer):
        file_instance = serializer.save()
        # Update Docker file
        self._write_to_docker(file_instance)

    def perform_destroy(self, instance):
        # Delete from Docker
        self._delete_from_docker(instance)
        instance.delete()

    def _write_to_docker(self, file_instance):
        """Write file content to Docker volume using docker exec"""
        docker_path = file_instance.full_docker_path

        try:
            # Create parent directories
            parent_dir = os.path.dirname(docker_path)
            subprocess.run(
                ["docker", "exec", DOCKER_CONTAINER, "mkdir", "-p", parent_dir],
                check=True,
                capture_output=True
            )

            if file_instance.file_type == WorkspaceFile.FILE:
                # Write file content
                subprocess.run(
                    ["docker", "exec", "-i", DOCKER_CONTAINER, "tee", docker_path],
                    input=file_instance.content.encode(),
                    check=True,
                    capture_output=True
                )
            else:
                # Create directory
                subprocess.run(
                    ["docker", "exec", DOCKER_CONTAINER, "mkdir", "-p", docker_path],
                    check=True,
                    capture_output=True
                )
        except subprocess.CalledProcessError as e:
            print(f"Error writing to Docker: {e.stderr.decode() if e.stderr else str(e)}")

    def _delete_from_docker(self, file_instance):
        """Delete file from Docker volume using docker exec"""
        docker_path = file_instance.full_docker_path

        try:
            if file_instance.file_type == WorkspaceFile.FILE:
                subprocess.run(
                    ["docker", "exec", DOCKER_CONTAINER, "rm", "-f", docker_path],
                    check=True,
                    capture_output=True
                )
            else:
                subprocess.run(
                    ["docker", "exec", DOCKER_CONTAINER, "rm", "-rf", docker_path],
                    check=True,
                    capture_output=True
                )
        except subprocess.CalledProcessError as e:
            print(f"Error deleting from Docker: {e.stderr.decode() if e.stderr else str(e)}")


class MeshViewSet(viewsets.ModelViewSet):
    """
    API endpoint for managing custom mesh files
    """
    serializer_class = CustomMeshSerializer
    permission_classes = [IsAuthenticated]

    def get_queryset(self):
        canvas_id = self.kwargs.get("canvas_pk")
        return CustomMesh.objects.filter(
            canvas_id=canvas_id, canvas__user=self.request.user
        )

    @action(detail=False, methods=["post"])
    def upload(self, request, canvas_pk=None):
        """
        Upload a custom mesh file (.stl, .dae, .obj)

        Request body (multipart/form-data):
        {
            "file": <file>,
            "name": "optional_custom_name"
        }
        """
        canvas = get_object_or_404(Canvas, id=canvas_pk, user=request.user)

        uploaded_file = request.FILES.get("file")
        if not uploaded_file:
            return Response(
                {"error": "No file provided"},
                status=status.HTTP_400_BAD_REQUEST
            )

        # Validate file extension
        valid_extensions = [".stl", ".dae", ".obj", ".STL", ".DAE", ".OBJ"]
        file_name = uploaded_file.name
        file_ext = file_name[file_name.rfind("."):] if "." in file_name else ""

        if file_ext not in valid_extensions:
            return Response(
                {"error": "Invalid file type. Supported: .stl, .dae, .obj"},
                status=status.HTTP_400_BAD_REQUEST
            )

        # Use provided name or file name
        mesh_name = request.data.get("name") or file_name

        # Check for duplicate name in this canvas
        if CustomMesh.objects.filter(canvas=canvas, name=mesh_name).exists():
            return Response(
                {"error": f"Mesh with name '{mesh_name}' already exists"},
                status=status.HTTP_400_BAD_REQUEST
            )

        # Create mesh instance
        mesh = CustomMesh.objects.create(
            canvas=canvas,
            user=request.user,
            name=mesh_name,
            file=uploaded_file,
            file_size=uploaded_file.size,
            file_type=file_ext.lower()
        )

        serializer = self.get_serializer(mesh)
        return Response(serializer.data, status=status.HTTP_201_CREATED)

    @action(detail=False, methods=["post"])
    def import_from_url(self, request, canvas_pk=None):
        """
        Import a mesh file from a URL

        Request body:
        {
            "url": "http://example.com/mesh.stl",
            "name": "optional_custom_name"
        }
        """
        canvas = get_object_or_404(Canvas, id=canvas_pk, user=request.user)

        url = request.data.get("url")
        if not url:
            return Response(
                {"error": "URL is required"},
                status=status.HTTP_400_BAD_REQUEST
            )

        try:
            # Download file from URL
            response = requests.get(url, timeout=30)
            response.raise_for_status()

            # Extract filename from URL
            url_filename = url.split("/")[-1].split("?")[0]

            # Validate file extension
            valid_extensions = [".stl", ".dae", ".obj", ".STL", ".DAE", ".OBJ"]
            file_ext = url_filename[url_filename.rfind("."):] if "." in url_filename else ""

            if file_ext not in valid_extensions:
                return Response(
                    {"error": "Invalid file type in URL. Supported: .stl, .dae, .obj"},
                    status=status.HTTP_400_BAD_REQUEST
                )

            # Use provided name or URL filename
            mesh_name = request.data.get("name") or url_filename

            # Check for duplicate name
            if CustomMesh.objects.filter(canvas=canvas, name=mesh_name).exists():
                return Response(
                    {"error": f"Mesh with name '{mesh_name}' already exists"},
                    status=status.HTTP_400_BAD_REQUEST
                )

            # Create mesh instance with downloaded content
            mesh = CustomMesh(
                canvas=canvas,
                user=request.user,
                name=mesh_name,
                file_size=len(response.content),
                file_type=file_ext.lower()
            )

            # Save file content
            mesh.file.save(url_filename, ContentFile(response.content), save=True)

            serializer = self.get_serializer(mesh)
            return Response(serializer.data, status=status.HTTP_201_CREATED)

        except requests.exceptions.RequestException as e:
            return Response(
                {"error": f"Failed to download file from URL: {str(e)}"},
                status=status.HTTP_400_BAD_REQUEST
            )
        except Exception as e:
            return Response(
                {"error": f"Failed to import mesh: {str(e)}"},
                status=status.HTTP_500_INTERNAL_SERVER_ERROR
            )


@api_view(["GET"])
def serve_mesh(request, canvas_id, filename):
    """
    Serve mesh files for URDF visualization
    Resolves package://workspace_{canvas_id}/meshes/{filename} URLs

    Public endpoint - no authentication required for mesh viewing
    """
    try:
        # Find the mesh file
        mesh = CustomMesh.objects.filter(
            canvas_id=canvas_id,
            file__icontains=filename
        ).first()

        if not mesh:
            return Response(
                {"error": "Mesh file not found"},
                status=status.HTTP_404_NOT_FOUND
            )

        # Determine content type based on file extension
        content_type_map = {
            '.stl': 'model/stl',
            '.dae': 'model/vnd.collada+xml',
            '.obj': 'model/obj',
        }
        content_type = content_type_map.get(mesh.file_type.lower(), 'application/octet-stream')

        # Return the file with appropriate content type
        from django.http import FileResponse
        response = FileResponse(mesh.file.open('rb'), content_type=content_type)

        # Add CORS headers to allow cross-origin requests from frontend
        response['Access-Control-Allow-Origin'] = '*'
        response['Access-Control-Allow-Methods'] = 'GET, OPTIONS'
        response['Access-Control-Allow-Headers'] = 'Content-Type'

        return response

    except Exception as e:
        return Response(
            {"error": f"Failed to serve mesh: {str(e)}"},
            status=status.HTTP_500_INTERNAL_SERVER_ERROR
        )


@api_view(["POST"])
@permission_classes([IsAuthenticated])
def execute_command(request, canvas_id):
    """
    Execute a command in the Docker container for this canvas

    Request body:
    {
        "command": "ls -la",
        "working_directory": "/workspaces/user/canvas_id" (optional)
    }

    Returns:
    {
        "output": "command output",
        "error": "error output if any",
        "exit_code": 0
    }
    """
    canvas = get_object_or_404(Canvas, id=canvas_id, user=request.user)

    command = request.data.get("command")
    if not command:
        return Response(
            {"error": "Command is required"}, status=status.HTTP_400_BAD_REQUEST
        )

    working_dir = request.data.get("working_directory", canvas.docker_path)

    # Execute command in Docker container
    try:
        # Execute command inside Docker container with working directory
        result = subprocess.run(
            [
                "docker", "exec", "-w", working_dir,
                DOCKER_CONTAINER,
                "bash", "-c", command
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )

        return Response(
            {
                "output": result.stdout,
                "error": result.stderr,
                "exit_code": result.returncode,
                "command": command,
                "working_directory": working_dir,
            }
        )
    except subprocess.TimeoutExpired:
        return Response(
            {"error": "Command execution timeout (30s)"},
            status=status.HTTP_408_REQUEST_TIMEOUT,
        )
    except Exception as e:
        return Response(
            {"error": str(e)}, status=status.HTTP_500_INTERNAL_SERVER_ERROR
        )
