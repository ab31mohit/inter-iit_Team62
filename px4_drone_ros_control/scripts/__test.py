import subprocess

def execute_command(command):
    """Execute a single command and return its output"""
    try:
        result = subprocess.run(
            command,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        return result.stdout, result.stderr
    except Exception as e:
        return '', str(e)

def main():
    # List of commands to execute
    commands = [
        "pwd",              # Current directory
        "date",            # Current date and time
        "whoami",          # Current user
        "ls -l",           # List files
        "df -h",           # Disk space
        "free -h",         # Memory usage
        "ps aux | head -5"  # Top 5 processes
    ]
    
    # Execute each command
    for command in commands:
        print(f"\nExecuting: {command}")
        output, error = execute_command(command)
        
        if output:
            print(output.strip())
        if error:
            print(f"Error: {error.strip()}")

if __name__ == "__main__":
    main()