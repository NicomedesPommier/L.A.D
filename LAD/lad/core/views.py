"""Core views for utility endpoints."""
from django.http import JsonResponse
from .ip_config import load_network_config

def network_config_view(request):
    cfg = load_network_config()
    return JsonResponse(cfg.as_dict())
