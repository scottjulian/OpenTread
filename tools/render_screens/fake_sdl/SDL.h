// Minimal stand-in for <SDL.h> so LovyanGFX builds headless on a desktop:
// it selects LovyanGFX's SDL platform headers (heap/millis declarations)
// without linking SDL. Only the types those headers name are declared; the
// SDL panel itself is never compiled or used.
#pragma once
#define SDL_h_
typedef struct SDL_Window SDL_Window;
typedef struct SDL_Renderer SDL_Renderer;
typedef struct SDL_Texture SDL_Texture;
typedef struct SDL_mutex SDL_mutex;
typedef enum { KMOD_NONE = 0 } SDL_Keymod;
typedef enum { SDLK_UNKNOWN = 0 } SDL_KeyCode;
