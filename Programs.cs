using System.Numerics;
using Raylib_cs;

namespace Sandbox
{
    public static partial class Programs
    {
        public static void HelloWorld()
        {
            Raylib.InitWindow(800, 480, "Hello World");
            Raylib.SetTargetFPS(30);

            while (!Raylib.WindowShouldClose())
            {
                Raylib.BeginDrawing();
                Raylib.ClearBackground(Color.WHITE);

                Raylib.DrawText("Hello, world!", 12, 12, 20, Color.BLACK);
                Raylib.DrawText("Hello, world!", 12, 24, 20, Color.BLACK);

                Raylib.EndDrawing();
            }

            Raylib.CloseWindow();
        }

        public static void DrawCircle()
        {
            // data
            Color[] colors = new Color[4] { Color.BLACK, Color.RED, Color.GREEN, Color.BLUE };
            int currentColorIndex = 0;

            Raylib.InitWindow(800, 600, "Circle");
            while (!Raylib.WindowShouldClose())
            {
                // update
                Vector2 mousePos = Raylib.GetMousePosition();
                if (Raylib.IsMouseButtonDown(MouseButton.MOUSE_BUTTON_LEFT))
                    currentColorIndex = (currentColorIndex + 1) % 4;

                // render
                Raylib.BeginDrawing();
                Raylib.ClearBackground(Color.RAYWHITE);

                Raylib.DrawCircle((int)mousePos.X, (int)mousePos.Y, 50, colors[currentColorIndex]);

                Raylib.EndDrawing();
            }
            Raylib.CloseWindow();
        }

        public static void CameraRendering2D()
        {
            // data
            int screenWidth = 800;
            int screenHeight = 600;
            double targetFPS = 60;

            Rectangle player = new Rectangle(400, 300, 33, 67);
            Camera2D camera2d = new Camera2D(new Vector2(screenWidth, screenHeight) / 2, new Vector2(player.x, player.y), 0, 1);

            Raylib.InitWindow(screenWidth, screenHeight, "Camera render 2D");
            {
                Raylib.SetTargetFPS((int)targetFPS);
                while (!Raylib.WindowShouldClose())
                {
                    float deltaTime = Raylib.GetFrameTime();

                    // update
                    camera2d.target = new Vector2(player.x + player.width / 2, player.y + player.height / 2);
                    if (Raylib.IsKeyDown(KeyboardKey.KEY_A))
                        camera2d.rotation += (float)deltaTime * 100;
                    else if (Raylib.IsKeyDown(KeyboardKey.KEY_D))
                        camera2d.rotation -= (float)deltaTime * 100;

                    // render
                    Raylib.BeginDrawing();
                    {
                        Raylib.ClearBackground(Color.RAYWHITE);

                        Raylib.BeginMode2D(camera2d);
                        {
                            Raylib.DrawRectangle((int)player.x, (int)player.y, (int)player.width, (int)player.height, Color.RED);
                        }
                        Raylib.EndMode2D();
                    }
                    Raylib.DrawText($"deltaTime:{deltaTime}", 12, 12, 20, Color.BLACK);
                    Raylib.EndDrawing();
                }
            }
            Raylib.CloseWindow();
        }
    }
}
