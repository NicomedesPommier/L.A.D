# Django Backend Implementation Guide for Mesh Upload

This guide provides the complete Django backend implementation needed for the custom mesh upload feature in the Geometry node.

## Required API Endpoints

The frontend expects these endpoints to be available:

```
POST   /workspace/canvases/{canvas_id}/meshes/upload/
POST   /workspace/canvases/{canvas_id}/meshes/import/
GET    /workspace/canvases/{canvas_id}/meshes/
DELETE /workspace/canvases/{canvas_id}/meshes/{mesh_id}/
```

---

## 1. Django Model

Add this model to your workspace app (e.g., `workspace/models.py`):

```python
from django.db import models
from django.contrib.auth.models import User

class CustomMesh(models.Model):
    """
    Custom mesh files uploaded by users for their URDF robots
    """
    canvas = models.ForeignKey(
        'Canvas',
        on_delete=models.CASCADE,
        related_name='custom_meshes'
    )
    user = models.ForeignKey(
        User,
        on_delete=models.CASCADE,
        related_name='custom_meshes'
    )
    name = models.CharField(
        max_length=255,
        help_text="Display name for the mesh"
    )
    file = models.FileField(
        upload_to='meshes/%Y/%m/%d/',
        help_text="The actual mesh file (STL, DAE, OBJ)"
    )
    file_size = models.IntegerField(
        help_text="File size in bytes"
    )
    file_type = models.CharField(
        max_length=10,
        help_text="File extension (.stl, .dae, .obj)"
    )
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        ordering = ['-created_at']
        unique_together = ['canvas', 'name']

    def __str__(self):
        return f"{self.name} ({self.canvas.name})"

    @property
    def file_path(self):
        """
        Returns the package:// style path for URDF
        """
        return f"package://workspace_{self.canvas.id}/meshes/{self.file.name.split('/')[-1]}"
```

---

## 2. Django Serializer

Create a serializer (e.g., `workspace/serializers.py`):

```python
from rest_framework import serializers
from .models import CustomMesh

class CustomMeshSerializer(serializers.ModelSerializer):
    file_path = serializers.ReadOnlyField()

    class Meta:
        model = CustomMesh
        fields = [
            'id',
            'name',
            'file',
            'file_path',
            'file_size',
            'file_type',
            'created_at',
        ]
        read_only_fields = ['id', 'file_size', 'file_type', 'created_at']
```

---

## 3. Django Views

Create views in `workspace/views.py`:

```python
import os
import requests
from django.core.files.base import ContentFile
from rest_framework import viewsets, status
from rest_framework.decorators import action
from rest_framework.response import Response
from rest_framework.permissions import IsAuthenticated
from .models import Canvas, CustomMesh
from .serializers import CustomMeshSerializer

class MeshViewSet(viewsets.ModelViewSet):
    """
    ViewSet for managing custom mesh files
    """
    serializer_class = CustomMeshSerializer
    permission_classes = [IsAuthenticated]

    def get_queryset(self):
        """Filter meshes by canvas and user"""
        canvas_id = self.kwargs.get('canvas_pk')
        return CustomMesh.objects.filter(
            canvas_id=canvas_id,
            user=self.request.user
        )

    def perform_create(self, serializer):
        """Auto-set user and canvas when creating"""
        canvas_id = self.kwargs.get('canvas_pk')
        canvas = Canvas.objects.get(id=canvas_id, user=self.request.user)
        serializer.save(user=self.request.user, canvas=canvas)

    @action(detail=False, methods=['post'])
    def upload(self, request, canvas_pk=None):
        """
        Upload a mesh file

        POST /workspace/canvases/{canvas_id}/meshes/upload/
        Body (multipart/form-data):
          - file: <mesh_file>
          - name: <optional_name>
        """
        # Verify canvas ownership
        try:
            canvas = Canvas.objects.get(id=canvas_pk, user=request.user)
        except Canvas.DoesNotExist:
            return Response(
                {"error": "Canvas not found"},
                status=status.HTTP_404_NOT_FOUND
            )

        # Get uploaded file
        uploaded_file = request.FILES.get('file')
        if not uploaded_file:
            return Response(
                {"error": "No file provided"},
                status=status.HTTP_400_BAD_REQUEST
            )

        # Validate file type
        valid_extensions = ['.stl', '.dae', '.obj', '.STL', '.DAE', '.OBJ']
        file_ext = os.path.splitext(uploaded_file.name)[1]
        if file_ext not in valid_extensions:
            return Response(
                {"error": f"Invalid file type. Allowed: {', '.join(valid_extensions)}"},
                status=status.HTTP_400_BAD_REQUEST
            )

        # Get or generate name
        name = request.data.get('name', uploaded_file.name)

        # Check for duplicate name
        if CustomMesh.objects.filter(canvas=canvas, name=name).exists():
            return Response(
                {"error": f"A mesh named '{name}' already exists"},
                status=status.HTTP_400_BAD_REQUEST
            )

        # Create mesh record
        mesh = CustomMesh.objects.create(
            canvas=canvas,
            user=request.user,
            name=name,
            file=uploaded_file,
            file_size=uploaded_file.size,
            file_type=file_ext.lower()
        )

        serializer = self.get_serializer(mesh)
        return Response(serializer.data, status=status.HTTP_201_CREATED)

    @action(detail=False, methods=['post'])
    def import_from_url(self, request, canvas_pk=None):
        """
        Import a mesh from a URL

        POST /workspace/canvases/{canvas_id}/meshes/import/
        Body (JSON):
          {
            "url": "http://example.com/mesh.stl",
            "name": "optional_name"
          }
        """
        # Verify canvas ownership
        try:
            canvas = Canvas.objects.get(id=canvas_pk, user=request.user)
        except Canvas.DoesNotExist:
            return Response(
                {"error": "Canvas not found"},
                status=status.HTTP_404_NOT_FOUND
            )

        # Get URL
        url = request.data.get('url')
        if not url:
            return Response(
                {"error": "No URL provided"},
                status=status.HTTP_400_BAD_REQUEST
            )

        try:
            # Download file from URL
            response = requests.get(url, timeout=30)
            response.raise_for_status()

            # Get filename from URL
            filename = url.split('/')[-1]
            file_ext = os.path.splitext(filename)[1]

            # Validate file type
            valid_extensions = ['.stl', '.dae', '.obj', '.STL', '.DAE', '.OBJ']
            if file_ext not in valid_extensions:
                return Response(
                    {"error": f"Invalid file type. Allowed: {', '.join(valid_extensions)}"},
                    status=status.HTTP_400_BAD_REQUEST
                )

            # Get or generate name
            name = request.data.get('name', filename)

            # Check for duplicate name
            if CustomMesh.objects.filter(canvas=canvas, name=name).exists():
                return Response(
                    {"error": f"A mesh named '{name}' already exists"},
                    status=status.HTTP_400_BAD_REQUEST
                )

            # Create mesh record
            mesh = CustomMesh.objects.create(
                canvas=canvas,
                user=request.user,
                name=name,
                file_size=len(response.content),
                file_type=file_ext.lower()
            )

            # Save file content
            mesh.file.save(filename, ContentFile(response.content), save=True)

            serializer = self.get_serializer(mesh)
            return Response(serializer.data, status=status.HTTP_201_CREATED)

        except requests.RequestException as e:
            return Response(
                {"error": f"Failed to download file: {str(e)}"},
                status=status.HTTP_400_BAD_REQUEST
            )
```

---

## 4. URL Configuration

Update your `workspace/urls.py`:

```python
from django.urls import path, include
from rest_framework_nested import routers
from .views import CanvasViewSet, MeshViewSet

router = routers.DefaultRouter()
router.register(r'canvases', CanvasViewSet, basename='canvas')

# Nested router for meshes under canvases
canvases_router = routers.NestedDefaultRouter(router, r'canvases', lookup='canvas')
canvases_router.register(r'meshes', MeshViewSet, basename='canvas-meshes')

urlpatterns = [
    path('', include(router.urls)),
    path('', include(canvases_router.urls)),
]
```

---

## 5. Settings Configuration

Add to your `settings.py`:

```python
# Media files configuration
MEDIA_URL = '/media/'
MEDIA_ROOT = os.path.join(BASE_DIR, 'media')

# File upload settings
FILE_UPLOAD_MAX_MEMORY_SIZE = 10485760  # 10MB
DATA_UPLOAD_MAX_MEMORY_SIZE = 10485760  # 10MB

# Install required packages
# pip install drf-nested-routers requests
```

---

## 6. Migrations

Run migrations:

```bash
python manage.py makemigrations
python manage.py migrate
```

---

## 7. Serve Media Files (Development)

In your main `urls.py`:

```python
from django.conf import settings
from django.conf.urls.static import static

urlpatterns = [
    # ... your other patterns
] + static(settings.MEDIA_URL, document_root=settings.MEDIA_ROOT)
```

---

## 8. Testing the API

### Upload Test:
```bash
curl -X POST \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -F "file=@/path/to/mesh.stl" \
  -F "name=my_custom_mesh" \
  http://localhost:8000/workspace/canvases/1/meshes/upload/
```

### Import from URL Test:
```bash
curl -X POST \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"url": "http://localhost:7000/qcar_description/meshes/QCarBody.stl", "name": "qcar_body"}' \
  http://localhost:8000/workspace/canvases/1/meshes/import/
```

### List Meshes Test:
```bash
curl -X GET \
  -H "Authorization: Bearer YOUR_TOKEN" \
  http://localhost:8000/workspace/canvases/1/meshes/
```

### Delete Mesh Test:
```bash
curl -X DELETE \
  -H "Authorization: Bearer YOUR_TOKEN" \
  http://localhost:8000/workspace/canvases/1/meshes/5/
```

---

## 9. Expected Response Formats

### Upload/Import Success:
```json
{
  "id": 1,
  "name": "my_custom_mesh",
  "file": "/media/meshes/2025/12/08/mesh.stl",
  "file_path": "package://workspace_1/meshes/mesh.stl",
  "file_size": 524288,
  "file_type": ".stl",
  "created_at": "2025-12-08T12:34:56Z"
}
```

### List Meshes Success:
```json
[
  {
    "id": 1,
    "name": "my_custom_mesh",
    "file": "/media/meshes/2025/12/08/mesh.stl",
    "file_path": "package://workspace_1/meshes/mesh.stl",
    "file_size": 524288,
    "file_type": ".stl",
    "created_at": "2025-12-08T12:34:56Z"
  }
]
```

---

## 10. Dependencies

Add to `requirements.txt`:

```
djangorestframework
drf-nested-routers
requests
Pillow  # If you want to handle image validation
```

Install with:
```bash
pip install -r requirements.txt
```

---

## Notes

1. **Security**: Add file size limits and virus scanning in production
2. **Storage**: Consider using cloud storage (S3, GCS) for production
3. **Permissions**: Ensure users can only access their own canvases
4. **CORS**: Configure CORS if frontend is on different domain
5. **Rate Limiting**: Add rate limiting for upload endpoints
6. **File Cleanup**: Consider adding a cleanup task for orphaned files

---

## Frontend Integration

Once the backend is implemented, the frontend will automatically work. The GeometryNode component will:
1. Upload files via the Upload tab
2. Import from URLs via the URL tab
3. Browse uploaded meshes via the Library tab
4. Use the returned `file_path` in URDF generation
