from fpdf import FPDF

# Crear un objeto PDF
pdf = FPDF()

# Añadir una página
pdf.add_page()

# Título
pdf.set_font('Arial', 'B', 16)
pdf.cell(200, 10, txt="Resumen de Proyectos de Investigación", ln=True, align='C')

# Espacio
pdf.ln(10)

# Primer Proyecto
pdf.set_font('Arial', 'B', 12)
pdf.cell(200, 10, txt="1. SEIA – Semillero en Energías, Instrumentación y Automatización", ln=True)
pdf.set_font('Arial', '', 12)
pdf.multi_cell(0, 10, txt="""Grupo de Investigación VOLTA
Universidad Militar Nueva Granada – Programa de Ingeniería en Mecatrónica

Ideas que llaman la atención:
- Aplicación de generadores de vórtice (VGs) en turbinas tipo Darrieus para mejorar su rendimiento aerodinámico, lo que representa una alternativa innovadora para optimizar el aprovechamiento de energía eólica.
- Enfoque combinado entre simulación computacional (CFD con ANSYS Fluent) y validación experimental para reducir errores y obtener resultados más cercanos a la realidad.

Objetivos del proyecto y alcance:
Objetivo General:
- Evaluar el comportamiento de generadores de vórtice en turbinas de eje vertical tipo Darrieus mediante análisis computacional y validación experimental para mejorar el rendimiento del sistema.

Objetivos Específicos:
- Implementar y analizar diferentes geometrías de generadores de vórtice en simulaciones CFD.
- Comparar resultados numéricos con pruebas experimentales.
- Validar la metodología propuesta como una alternativa de mejora del desempeño aerodinámico en turbinas tipo Darrieus.

Alcance del Proyecto:
- El proyecto abarca desde el diseño asistido por computador (CAD) de la turbina y los VGs, simulaciones CFD utilizando el modelo de turbulencia SST, hasta ensayos experimentales.
- Se logró demostrar cómo el uso de VGs puede reducir zonas de separación de flujo, mejorar la estabilidad aerodinámica y aumentar la eficiencia de la turbina.
- Tiene implicaciones en el desarrollo de tecnologías más eficientes para la generación de energía renovable.
""")

# Espacio
pdf.ln(10)

# Segundo Proyecto
pdf.set_font('Arial', 'B', 12)
pdf.cell(200, 10, txt="2. SEIA (Semillero en Energías, Instrumentación y Automatización)", ln=True)
pdf.set_font('Arial', '', 12)
pdf.multi_cell(0, 10, txt="""Grupo de investigación VOLTA
Universidad Militar Nueva Granada – Programa de Ingeniería en Mecatrónica

Ideas que llaman la atención:
- Evaluación de la influencia del diseño de plantillas personalizadas a partir de escaneo 3D, orientadas a mejorar la postura y distribución de presión en personas con pie plano.
- Uso combinado de tecnología de escaneo 3D, software de análisis biomecánico y diseño computacional para crear plantillas ortopédicas personalizadas.

Objetivos del proyecto y alcance:
Objetivo General:
- Evaluar el efecto de un control del equilibrio postural en plantillas personalizadas mediante escaneo 3D, para personas con pie plano flexible.

Objetivos Específicos:
- Identificar los puntos de presión plantar y su distribución en personas con pie plano.
- Diseñar plantillas personalizadas que optimicen la postura del usuario.
- Realizar simulaciones biomecánicas para evaluar el impacto de las plantillas diseñadas.
- Comparar los resultados de presión plantar antes y después del uso de las plantillas personalizadas.
""")

# Espacio
pdf.ln(10)

# Tercer Proyecto
pdf.set_font('Arial', 'B', 12)
pdf.cell(200, 10, txt="3. GROST – Grupo de Restauración de Sistemas Tropicales", ln=True)
pdf.set_font('Arial', '', 12)
pdf.multi_cell(0, 10, txt="""IDEAS – Investigación en Desarrollo Sostenible
Universidad Militar Nueva Granada

Ideas que llaman la atención:
- Integración de inteligencia artificial y machine learning para el diseño de estrategias de restauración ecológica.
- Enfoque interdisciplinario que combina drones, análisis de conectividad ecológica y herramientas digitales para la toma de decisiones en la restauración del ecosistema.

Objetivos del proyecto y alcance:
Objetivo General:
- Establecer estrategias de restauración ecológica en paisajes fragmentados a través del uso de tecnologías emergentes y análisis de conectividad ecológica, para promover la conservación de la biodiversidad.

Objetivos Específicos:
- Recolectar datos de campo con drones para mapear zonas críticas.
- Analizar la conectividad ecológica del paisaje.
- Aplicar inteligencia artificial y machine learning para proponer zonas óptimas de restauración.
- Diseñar estrategias de restauración que integren datos ecológicos y urbanos.
""")

# Espacio
pdf.ln(10)

# Cuarto Proyecto
pdf.set_font('Arial', 'B', 12)
pdf.cell(200, 10, txt="4. ENSO – Energías Sostenibles", ln=True)
pdf.set_font('Arial', '', 12)
pdf.multi_cell(0, 10, txt="""Universidad Militar Nueva Granada

Ideas que llaman la atención:
- Diseño y construcción de un banco de pruebas para comparar la eficiencia térmica de distintas placas colectoras solares con geometrías y materiales variados.
- Uso de software de simulación (SolidWorks) para validar los diseños antes de su implementación práctica.

Objetivos del proyecto y alcance:
Objetivo General:
- Desarrollar un banco de prueba de placas colectoras para evaluar su eficiencia en sistemas de captación de energía solar térmica.

Objetivos Específicos:
- Desarrollar el banco de pruebas.
- Realizar experimentación y análisis comparativo de placas colectoras con diferentes configuraciones geométricas, materiales y estructuras.
""")

# Espacio
pdf.ln(10)

# Quinto Proyecto
pdf.set_font('Arial', 'B', 12)
pdf.cell(200, 10, txt="5. DAVINCI - INMED", ln=True)
pdf.set_font('Arial', '', 12)
pdf.multi_cell(0, 10, txt="""Grupo de investigación: DAVINCI
Semillero de investigación: INMED
Línea de investigación: Ingeniería – Rehabilitación
Universidad Militar Nueva Granada

Ideas que llaman la atención:
- El uso del Taichí como herramienta terapéutica no farmacológica para la rehabilitación funcional del hombro en adultos.
- La combinación de análisis cuantitativos (movimientos con software de captura) y cualitativos (encuestas de satisfacción) para evaluar los resultados del tratamiento.

Objetivos del proyecto y alcance:
Objetivo General:
- Evaluar el efecto del Taichí como herramienta terapéutica en la rehabilitación funcional del hombro en hombres adultos.

Objetivos Específicos:
- Comparar los rangos de movimiento antes y después de la intervención.
- Evaluar la percepción subjetiva de los pacientes sobre el tratamiento mediante encuestas.
- Analizar los datos cuantitativos y cualitativos para validar la eficacia del Taichí como tratamiento complementario.
""")

# Guardar el archivo PDF
output_path = "/mnt/data/Resumen_Proyectos_Investigacion.pdf"
pdf.output(output_path)

output_path
