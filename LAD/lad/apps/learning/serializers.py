# apps/learning/serializers.py
from rest_framework import serializers
from .models import Unit, Level, Objective, UserProgress, ObjectiveProgress

class ObjectiveProgressSerializer(serializers.Serializer):
    achieved = serializers.BooleanField()
    achieved_at = serializers.DateTimeField(allow_null=True)

class ObjectiveSerializer(serializers.ModelSerializer):
    description = serializers.SerializerMethodField()
    user_progress = serializers.SerializerMethodField()

    class Meta:
        model = Objective
        fields = ("code", "description", "points", "user_progress")

    def get_description(self, obj):
        return getattr(obj, "description", None) or getattr(obj, "Description", "")

    def get_user_progress(self, obj):
        # Primero: usar prefetch (si está)
        if hasattr(obj, "_user_obj_prog") and obj._user_obj_prog:
            op = obj._user_obj_prog[0]
            return {"achieved": op.achieved, "achieved_at": op.achieved_at}
        # Fallback: query como antes
        user = self.context["request"].user
        try:
            op = ObjectiveProgress.objects.get(user=user, objective=obj)
            return {"achieved": op.achieved, "achieved_at": op.achieved_at}
        except ObjectiveProgress.DoesNotExist:
            return {"achieved": False, "achieved_at": None}

class LevelSerializer(serializers.ModelSerializer):
    objectives = ObjectiveSerializer(many=True, read_only=True)
    user_progress = serializers.SerializerMethodField()

    class Meta:
        model = Level
        fields = ("slug", "title", "order", "is_active", "objectives", "user_progress")

    def get_user_progress(self, obj):
        # Usar prefetch si existe
        if hasattr(obj, "_user_level_prog") and obj._user_level_prog:
            up = obj._user_level_prog[0]
            return {"completed": up.completed, "completed_at": up.completed_at, "score": up.score}
        # Fallback
        user = self.context["request"].user
        try:
            up = UserProgress.objects.get(user=user, level=obj)
            return {"completed": up.completed, "completed_at": up.completed_at, "score": up.score}
        except UserProgress.DoesNotExist:
            return {"completed": False, "completed_at": None, "score": 0}



class UnitSerializer(serializers.ModelSerializer):
    levels = LevelSerializer(many=True, read_only=True)
    user_progress = serializers.SerializerMethodField()

    class Meta:
        model = Unit
        fields = ("slug", "title", "order", "is_active", "levels", "user_progress")

    def get_user_progress(self, obj):
        from .models import UnitProgress
        user = self.context["request"].user
        try:
            up = UnitProgress.objects.get(user=user, unit=obj)
            return {"completed": up.completed, "completed_at": up.completed_at, "score": up.score}
        except UnitProgress.DoesNotExist:
            # opcional: derivar de levels si quieres
            all_levels = list(getattr(obj, "levels", []).all()) if hasattr(obj, "levels") else []
            completed = len(all_levels) > 0 and all(
                getattr(l, "_user_level_prog", [None])[0].completed
                if hasattr(l, "_user_level_prog") and l._user_level_prog
                else False
                for l in all_levels
            )
            return {"completed": completed, "completed_at": None, "score": 0}