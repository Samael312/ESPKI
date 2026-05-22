import re
import sys
import time
import serial
from rich.live import Live
from rich.table import Table
from rich.console import Console

# Configuración
PORT = "COM7"
BAUD = 9600

console = Console()

# Estado inicial con estilos por defecto
state = {
    "cycle": "0",
    "heap": "0 KB",
    "read": "0",
    "skip": "0",
    "errors": "[green]0[/green]",
    "mqtt": "[yellow]Disconnected ✗[/yellow]",
    "modbus": "[yellow]Idle[/yellow]"
}

def generate_table() -> Table:
    """Genera una tabla estilizada con los datos actuales."""
    table = Table(
        title="[bold blue]📡 Kiconex Box Lite — Live Monitor[/bold blue]", 
        show_header=True, 
        header_style="bold magenta",
        box=None
    )
    table.add_column("Métrica", style="cyan", justify="right")
    table.add_column("Valor", style="white", justify="left")
    
    for key, value in state.items():
        # Formateamos visualmente la clave para que no parezca una variable interna
        display_key = key.replace("_", " ").title()
        table.add_row(display_key, str(value))
        
    return table

def main():
    console.print(f"[yellow]Intentando conectar a {PORT} a {BAUD} baudios...[/yellow]")
    
    try:
        with serial.Serial(PORT, BAUD, timeout=0.1) as ser, Live(generate_table(), refresh_per_second=10) as live:
            console.print(f"[green]✔ Conectado exitosamente a {PORT}[/green]\n")
            
            while True:
                if not ser.in_waiting:
                    time.sleep(0.01)  # Pequeño respiro para no saturar la CPU
                    continue
                    
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                # 1. Ciclo de Poll
                if m := re.search(r"poll cycle_s=(\d+)", line):
                    state["cycle"] = f"[bold white]{m.group(1)}s[/bold white]"
                    state["modbus"] = "[green]Polling...[/green]"

                # 2. Memoria Heap
                if m := re.search(r"heap=(\d+)", line):
                    kb = int(m.group(1)) // 1024
                    # Si queda poca RAM (ej. < 40KB), lo pone en naranja
                    color = "orange3" if kb < 40 else "green"
                    state["heap"] = f"[{color}]{kb} KB[/{color}]"

                # 3. Resultados de lectura Modbus
                if m := re.search(r"poll done: read=(\d+) errors=(\d+) skipped=(\d+)", line):
                    read, errors, skip = m.group(1, 2, 3)
                    state["read"] = read
                    state["skip"] = skip
                    state["modbus"] = "[cyan]Done[/cyan]"
                    
                    # Si hay errores, los resalta en rojo chillón
                    if int(errors) > 0:
                        state["errors"] = f"[bold red]{errors} ⚠[/bold red]"
                    else:
                        state["errors"] = "[green]0[/green]"

                # 4. Estado MQTT
                if "mqtt=connected" in line or "MQTT_EVENT_CONNECTED" in line:
                    state["mqtt"] = "[bold green]✓ Connected[/bold green]"
                elif "mqtt=disconnected" in line or "MQTT_EVENT_DISCONNECTED" in line:
                    state["mqtt"] = "[bold red]✗ Disconnected[/bold red]"

                # Actualiza la interfaz gráfica en la terminal
                live.update(generate_table())

    except serial.SerialException as e:
        console.print(f"\n[bold red]Error de Puerto Serial:[/bold red] {e}")
        console.print("[yellow]Asegúrate de que el ESP-IDF Monitor o VS Code no tengan el puerto abierto.[/yellow]")
    except KeyboardInterrupt:
        console.print("\n[bold green]Monitor cerrado por el usuario. ¡Hasta luego![/bold green]")
        sys.exit(0)

if __name__ == "__main__":
    main()