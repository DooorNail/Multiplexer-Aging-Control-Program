import os
import requests
import importlib.util
from pathlib import Path
import subprocess

class AgingControlUpdater:
    def __init__(self):
        self.script_url = "https://raw.githubusercontent.com/DooorNail/Multiplexer-Aging-Control-Program/refs/heads/main/Aging%20Control%20SPF50.py"
        self.script_name = "Aging Control SPF50.py"
        self.cache_dir = Path("aging_control_cache")
        self.cache_path = self.cache_dir / self.script_name
        
        # Ensure cache directory exists
        self.cache_dir.mkdir(exist_ok=True)
    
    def _download_script(self):
        """Download the script from GitHub"""
        try:
            print("Attempting to download latest version...")
            response = requests.get(self.script_url, timeout=10)
            response.raise_for_status()
            return response.text
        except (requests.RequestException, requests.Timeout) as e:
            print(f"Download failed: {e}")
            return None
    
    def _cache_script(self, script_content):
        """Cache the script content to a local file"""
        try:
            with open(self.cache_path, 'w', encoding='utf-8') as f:
                f.write(script_content)
            return True
        except IOError as e:
            print(f"Failed to cache script: {e}")
            return False
    
    def _load_cached_script(self):
        """Load the script from cache if available"""
        if self.cache_path.exists():
            try:
                print("Loading cached version...")
                with open(self.cache_path, 'r', encoding='utf-8') as f:
                    return f.read()
            except IOError as e:
                print(f"Failed to read cached script: {e}")
        return None
    
    def _run_script(self, script_content):
        """Execute the Python script content"""
        try:
            # Create a module specification and module
            spec = importlib.util.spec_from_loader(
                self.script_name.replace('.py', ''),
                loader=None,
                origin=str(self.cache_path)
            )
            module = importlib.util.module_from_spec(spec)
            
            # Execute the script content in the module's namespace
            exec(script_content, module.__dict__)
            return True
        except Exception as e:
            print(f"Error executing script: {e}")
            return False
    
    def _run_in_new_window(self):
        """Run the script in a new terminal window"""
        try:

            # Windows - start in new cmd window
            subprocess.Popen(f'start cmd /k python "{self.cache_path}"', shell=True)
            
            return True
        except Exception as e:
            print(f"Failed to launch new window: {e}")
            return False
    
    def run(self):
        """Main method to download and run the script with fallback to cache"""
        # Try to download the latest version
        script_content = self._download_script()
        
        if script_content and self._cache_script(script_content):
            print("Successfully downloaded latest version")
        elif self.cache_path.exists():
            print("Using cached version")
        else:
            print("Error: No script available")
            input("Press Enter to exit...")
            return

        # Run in new window
        if self._run_in_new_window():
            print(f"Launched script in new window. You can close this window.")
        else:
            print("Failed to launch script - trying to run in current window...")
            os.system(f'python "{self.cache_path}"')


if __name__ == "__main__":
    updater = AgingControlUpdater()
    updater.run()