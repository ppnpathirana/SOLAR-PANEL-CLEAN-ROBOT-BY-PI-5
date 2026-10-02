import os, re

path = r'C:\Users\Pasindu\.gemini\antigravity\scratch\CleanBot_v1\app.py'
with open(path, 'r', encoding='utf-8') as f:
    c = f.read()

# 1. Add logging and PDF dependencies
c = c.replace('import os, sys, time, math, json, csv, io, sqlite3, threading, atexit', 
              'import os, sys, time, math, json, csv, io, sqlite3, threading, atexit, logging\nfrom reportlab.pdfgen import canvas\nfrom reportlab.lib.pagesizes import letter')

# 2. Setup logger
logger_setup = '''
# ── Logging Setup ──
os.makedirs('logs', exist_ok=True)
os.makedirs('reports/damage', exist_ok=True)
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    handlers=[logging.FileHandler('logs/cleanbot.log'), logging.StreamHandler()])
logger = logging.getLogger(__name__)
'''
c = c.replace("app.config['SECRET_KEY'] = 'cb42'", logger_setup + "\napp.config['SECRET_KEY'] = 'cb42'")

# Replace prints with logger.info or logger.error
c = re.sub(r'print\((f?[\'"].*?[\'"])\)', r'logger.info(\1)', c)
c = c.replace('logger.info(f"⚠', 'logger.warning(f"⚠')
c = c.replace('logger.info(f"❌', 'logger.error(f"❌')

# 3. Add damage snapshotting in vision loop
vision_mod = '''
                        if 'damage' in nm: 
                            damage_count += 1
                            if damage_count % 30 == 1: # Save 1 snapshot approx every second of seeing damage
                                ts = datetime.now().strftime('%Y%m%d_%H%M%S')
                                cv2.imwrite(f'reports/damage/dmg_{ts}.jpg', frame)
                                logger.warning(f'Damage snapshot saved: dmg_{ts}.jpg')
'''
c = re.sub(r"if 'damage' in nm:\s*damage_count \+= 1", vision_mod.strip(), c)

# 4. Add PDF Export endpoint
pdf_endpoint = '''
@app.route('/history/pdf')
def history_pdf():
    rows = _get_history()
    out = io.BytesIO()
    c = canvas.Canvas(out, pagesize=letter)
    c.setFont("Helvetica-Bold", 20)
    c.drawString(50, 750, "CleanBot v1.0 - Session History Report")
    c.setFont("Helvetica", 12)
    y = 700
    for r in rows:
        c.drawString(50, y, f"Date: {r['date']} | Start: {r['start']} | Dur: {r['duration']} | Area: {r['area']}m2 | Dirt: {r['dirt']}% | Dmg: {r['damage']}")
        y -= 20
        if y < 50:
            c.showPage()
            c.setFont("Helvetica", 12)
            y = 750
    c.save()
    out.seek(0)
    return send_file(out, mimetype='application/pdf', as_attachment=True, download_name=f"cleanbot_report_{datetime.now().strftime('%Y%m%d')}.pdf")
'''
c = c.replace("@app.route('/history/csv')", pdf_endpoint + "\n@app.route('/history/csv')")

with open(path, 'w', encoding='utf-8') as f:
    f.write(c)

# 5. Add reportlab to requirements
with open(r'C:\Users\Pasindu\.gemini\antigravity\scratch\CleanBot_v1\requirements.txt', 'a', encoding='utf-8') as f:
    f.write('reportlab>=4.0.0\n')
