using System.Diagnostics;
using Esc.Bridge;
using Esc.Web.Components;
using Esc.Web.Services;
using Microsoft.AspNetCore.Hosting.Server;
using Microsoft.AspNetCore.Hosting.Server.Features;

var builder = WebApplication.CreateBuilder(args);

// Add services to the container.
builder.Services.AddRazorComponents()
    .AddInteractiveServerComponents();
builder.Services.AddEscBridge();
builder.Services.AddSingleton<ModularControlProvider>();

var app = builder.Build();
bool openBrowserOnStartup = builder.Configuration.GetValue(
    "GuiLauncher:OpenBrowserOnStartup",
    !app.Environment.IsDevelopment());

// Configure the HTTP request pipeline.
if (!app.Environment.IsDevelopment())
{
    app.UseExceptionHandler(exceptionHandlerApp =>
    {
        exceptionHandlerApp.Run(async context =>
        {
            context.Response.StatusCode = 500;
            context.Response.ContentType = "text/html; charset=utf-8";
            await context.Response.WriteAsync("""
                <!DOCTYPE html>
                <html lang="es">
                <head><meta charset="utf-8"><title>Error - ESC Console</title>
                <style>body{background:#071016;color:#d8e8ef;font-family:"Cascadia Mono",Consolas,monospace;display:grid;place-items:center;min-height:100vh;margin:0}
                .e-box{max-width:480px;border:1px solid #345269;border-left:3px solid #ff6269;background:rgba(16,29,39,0.91);padding:2rem}
                h1{color:#ff9ba0;font-size:1.15rem;margin:0 0 1rem;letter-spacing:-0.03em}
                p{color:#7e9aae;margin:0 0 1.2rem;line-height:1.5}
                a{color:#49c6ed;text-decoration:none}
                a:hover{text-decoration:underline}</style></head>
                <body><div class="e-box"><h1>Error del servidor</h1>
                <p>No se pudo procesar la solicitud. Revisa que el ESC est\u00E9 conectado y vuelve a intentarlo.</p>
                <a href="/">Volver al dashboard</a></div></body></html>
                """);
        });
    });
}
app.UseStatusCodePagesWithReExecute("/not-found", createScopeForStatusCodePages: true);
app.UseWebSockets();
app.UseAntiforgery();

app.MapStaticAssets();
app.MapEscBridgeEndpoints();
app.MapRazorComponents<App>()
    .AddInteractiveServerRenderMode();

if (openBrowserOnStartup)
{
    app.Lifetime.ApplicationStarted.Register(() =>
    {
        string? address = app.Services
            .GetRequiredService<IServer>()
            .Features
            .Get<IServerAddressesFeature>()?
            .Addresses
            .FirstOrDefault();

        if (string.IsNullOrWhiteSpace(address))
        {
            return;
        }

        try
        {
            Process.Start(new ProcessStartInfo
            {
                FileName = address,
                UseShellExecute = true
            });
        }
        catch
        {
            // Si no se puede abrir el navegador, el servidor igual queda levantado.
        }
    });
}

app.Run();
