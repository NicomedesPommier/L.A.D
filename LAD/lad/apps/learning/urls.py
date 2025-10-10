# apps/learning/urls.py
from django.urls import path, include
from rest_framework.routers import DefaultRouter
from .views import UnitViewSet, LevelViewSet, hit_objective

router = DefaultRouter()
router.register(r"units", UnitViewSet, basename="unit")
router.register(r"levels", LevelViewSet, basename="level")

# Mapea explícito a las acciones del ViewSet
level_complete = LevelViewSet.as_view({"post": "complete"})
level_reset = LevelViewSet.as_view({"post": "reset"})

urlpatterns = [
    path("", include(router.urls)),
    path("progress/objective/<str:code>/hit/", hit_objective),
    # 👇 Rutas explícitas equivalentes a las del router
    path("levels/<slug:slug>/complete/", level_complete, name="level-complete"),
    path("levels/<slug:slug>/reset/", level_reset, name="level-reset"),
]
