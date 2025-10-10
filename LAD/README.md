
Este directorio está reservado para el backend y los servicios auxiliares que alimentan a la aplicación React.

## Qué deberías incluir aquí

- Código fuente del API (por ejemplo, Django, FastAPI o cualquier framework que exponga los endpoints REST).
- Archivos de configuración de infraestructura (Dockerfiles, `docker-compose.yml`, scripts de migraciones, etc.).
- Documentación específica del backend (diagramas, fixtures, notas de despliegue).

> ℹ️ **Sugerencia:** Si tu backend vive en un repositorio distinto, puedes agregarlo aquí como submódulo (`git submodule add <url> lad`). Así mantienes sincronizado el código sin duplicarlo.

## Cómo asegurarte de que el directorio se publique en GitHub

1. Verifica que no exista un repositorio Git independiente dentro de `lad/`. Si hay una carpeta `.git`, elimínala o convierte el backend en un submódulo explícito.
2. Ejecuta `git status` desde la raíz del proyecto y confirma que los archivos del backend aparecen como *untracked* o *modified*.
3. Agrega los archivos al commit: `git add lad`.
4. Confirma que `.gitignore` no esté filtrando los archivos que necesitas subir. Agrega excepciones si hiciera falta.
5. Realiza el commit y `git push` normalmente.

Si requieres mantener archivos temporales o secretos fuera del control de versiones, utiliza un `.env` local y evita subirlo (ya está ignorado por el `.gitignore` del proyecto).