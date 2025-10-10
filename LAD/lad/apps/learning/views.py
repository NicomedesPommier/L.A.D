# apps/learning/views.py
from django.utils import timezone
from django.db.models import Sum, Prefetch
from rest_framework.decorators import api_view, permission_classes


from rest_framework import permissions, viewsets, decorators, response, status

from .models import (
    Unit, Level, Objective,
    UserProgress, ObjectiveProgress, UnitProgress
)
from .serializers import UnitSerializer, LevelSerializer

class UnitViewSet(viewsets.ReadOnlyModelViewSet):
    serializer_class = UnitSerializer
    permission_classes = [permissions.IsAuthenticated]
    lookup_field = "slug"
    lookup_value_regex = r"[^/]+"

    def get_queryset(self):
        user = self.request.user
        # Prefetch ObjectiveProgress del usuario dentro de cada Objective
        user_obj_prog_qs = ObjectiveProgress.objects.filter(user=user)
        # Prefetch UserProgress del usuario por level
        user_lvl_prog_qs = UserProgress.objects.filter(user=user)

        return (
            Unit.objects.filter(is_active=True)
            .order_by("order")
            .prefetch_related(
                Prefetch(
                    "levels",
                    queryset=Level.objects.filter(is_active=True)
                    .order_by("order")
                    .prefetch_related(
                        Prefetch(
                            "objectives",
                            queryset=Objective.objects.all().order_by("id")
                            .prefetch_related(
                                Prefetch(
                                    "objectiveprogress_set",
                                    queryset=user_obj_prog_qs,
                                    to_attr="_user_obj_prog"    # ← lista en obj._user_obj_prog
                                )
                            ),
                        ),
                        Prefetch(
                            "userprogress_set",
                            queryset=user_lvl_prog_qs,
                            to_attr="_user_level_prog"      # ← lista en level._user_level_prog
                        ),
                    ),
                )
            )
        )

class LevelViewSet(viewsets.ReadOnlyModelViewSet):
    serializer_class = LevelSerializer
    permission_classes = [permissions.IsAuthenticated]
    lookup_field = "slug"
    lookup_value_regex = r"[^/]+"

    def get_queryset(self):
        user = self.request.user
        user_obj_prog_qs = ObjectiveProgress.objects.filter(user=user)
        user_lvl_prog_qs = UserProgress.objects.filter(user=user)
        return (
            Level.objects.filter(is_active=True)
            .order_by("order")
            .prefetch_related(
                Prefetch(
                    "objectives",
                    queryset=Objective.objects.all().order_by("id")
                    .prefetch_related(
                        Prefetch(
                            "objectiveprogress_set",
                            queryset=user_obj_prog_qs,
                            to_attr="_user_obj_prog"
                        )
                    )
                ),
                Prefetch(
                    "userprogress_set",
                    queryset=user_lvl_prog_qs,
                    to_attr="_user_level_prog"
                ),
            )
            .select_related("unit")
        )

    # 👇👇👇 ESTAS TRES ACCIONES DEBEN ESTAR DENTRO DE LA CLASE 👇👇👇
    @decorators.action(detail=False, methods=["get"], url_path="progress/me")
    def my_progress(self, request):
        ser = self.get_serializer(self.get_queryset(), many=True, context={"request": request})
        return response.Response(ser.data)

    @decorators.action(detail=True, methods=["post"], url_path="complete")
    def complete(self, request, slug=None):
        level = self.get_object()
        up, _ = UserProgress.objects.get_or_create(user=request.user, level=level)

        total = (
            ObjectiveProgress.objects
            .filter(user=request.user, objective__level=level, achieved=True)
            .aggregate(Sum("objective__points"))["objective__points__sum"] or 0
        )

        if not up.completed:
            up.completed = True
            up.completed_at = timezone.now()
        up.score = total
        up.save()

        # actualizar UnitProgress si aplica
        if level.unit_id:
            levels = level.unit.levels.filter(is_active=True)
            uprog, _ = UnitProgress.objects.get_or_create(user=request.user, unit=level.unit)
            uprog.score = (
                UserProgress.objects
                .filter(user=request.user, level__in=levels)
                .aggregate(Sum("score"))["score__sum"] or 0
            )
            all_completed = (
                UserProgress.objects
                .filter(user=request.user, level__in=levels, completed=True)
                .count() == levels.count()
            )
            if all_completed and not uprog.completed:
                uprog.completed = True
                uprog.completed_at = timezone.now()
            if not all_completed and uprog.completed:
                uprog.completed = False
                uprog.completed_at = None
            uprog.save()

        return response.Response({"ok": True, "completed": up.completed, "score": up.score})

    @decorators.action(detail=True, methods=["post"], url_path="reset")
    def reset(self, request, slug=None):
        level = self.get_object()

        ObjectiveProgress.objects.filter(user=request.user, objective__level=level).delete()

        up, _ = UserProgress.objects.get_or_create(user=request.user, level=level)
        up.completed = False
        up.completed_at = None
        up.score = 0
        up.save()

        if level.unit_id:
            levels = level.unit.levels.filter(is_active=True)
            uprog, _ = UnitProgress.objects.get_or_create(user=request.user, unit=level.unit)
            uprog.score = (
                UserProgress.objects
                .filter(user=request.user, level__in=levels)
                .aggregate(Sum("score"))["score__sum"] or 0
            )
            all_completed = (
                UserProgress.objects
                .filter(user=request.user, level__in=levels, completed=True)
                .count() == levels.count()
            )
            if not all_completed:
                uprog.completed = False
                uprog.completed_at = None
            uprog.save()

        return response.Response({"ok": True, "level_reset": True}, status=status.HTTP_200_OK)

@api_view(["POST"])
@permission_classes([permissions.IsAuthenticated])
def hit_objective(request, code):
    from django.db.models import Sum
    from django.utils import timezone
    from .models import Objective, ObjectiveProgress, UserProgress

    try:
        obj = Objective.objects.get(code=code)
    except Objective.DoesNotExist:
        return response.Response({"detail": "Objective not found"}, status=status.HTTP_404_NOT_FOUND)

    op, _ = ObjectiveProgress.objects.get_or_create(user=request.user, objective=obj)
    if not op.achieved:
        op.achieved = True
        op.achieved_at = timezone.now()
        op.save()

    up, _ = UserProgress.objects.get_or_create(user=request.user, level=obj.level)
    up.score = (
        ObjectiveProgress.objects.filter(user=request.user, objective__level=obj.level, achieved=True)
        .aggregate(Sum("objective__points"))["objective__points__sum"] or 0
    )
    up.save()

    return response.Response({"ok": True, "score": up.score}, status=status.HTTP_200_OK)



@decorators.action(detail=False, methods=["get"], url_path="progress/me")
def my_progress(self, request):
    ser = self.get_serializer(self.get_queryset(), many=True, context={"request": request})
    return response.Response(ser.data)
@decorators.action(detail=True, methods=["post"], url_path="complete")
def complete(self, request, slug=None):
    level = self.get_object()
    up, _ = UserProgress.objects.get_or_create(user=request.user, level=level)
    total = (
        ObjectiveProgress.objects
        .filter(user=request.user, objective__level=level, achieved=True)
        .aggregate(Sum("objective__points"))["objective__points__sum"] or 0
    )
    if not up.completed:
        up.completed = True
        up.completed_at = timezone.now()
    up.score = total
    up.save()
    # actualizar UnitProgress si el level pertenece a una unidad
    if level.unit_id:
        levels = level.unit.levels.filter(is_active=True)
        uprog, _ = UnitProgress.objects.get_or_create(user=request.user, unit=level.unit)
        uprog.score = (
            UserProgress.objects
            .filter(user=request.user, level__in=levels)
            .aggregate(Sum("score"))["score__sum"] or 0
        )
        all_completed = (
            UserProgress.objects
            .filter(user=request.user, level__in=levels, completed=True)
            .count() == levels.count()
        )
        if all_completed and not uprog.completed:
            uprog.completed = True
            uprog.completed_at = timezone.now()
        if not all_completed and uprog.completed:
            uprog.completed = False
            uprog.completed_at = None
        uprog.save()
    return response.Response({"ok": True, "completed": up.completed, "score": up.score})
@decorators.action(detail=True, methods=["post"], url_path="reset")
def reset(self, request, slug=None):
    level = self.get_object()
    # borrar progreso de objetivos del usuario en este level
    ObjectiveProgress.objects.filter(user=request.user, objective__level=level).delete()
    up, _ = UserProgress.objects.get_or_create(user=request.user, level=level)
    up.completed = False
    up.completed_at = None
    up.score = 0
    up.save()
    # actualizar UnitProgress si aplica
    if level.unit_id:
        levels = level.unit.levels.filter(is_active=True)
        uprog, _ = UnitProgress.objects.get_or_create(user=request.user, unit=level.unit)
        uprog.score = (
            UserProgress.objects
            .filter(user=request.user, level__in=levels)
            .aggregate(Sum("score"))["score__sum"] or 0
        )
        all_completed = (
            UserProgress.objects
            .filter(user=request.user, level__in=levels, completed=True)
            .count() == levels.count()
        )
        if not all_completed:
            uprog.completed = False
            uprog.completed_at = None
        uprog.save()
    return response.Response({"ok": True, "level_reset": True}, status=status.HTTP_200_OK)