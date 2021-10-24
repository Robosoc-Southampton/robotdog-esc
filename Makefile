html-notes: design-notes.html

design-notes.html: design-notes.md
	pandoc -f markdown -t html -s --mathjax design-notes.md \
		-o design-notes.html \
		--metadata title="Design of a Sensorless FOC brushless motor driver"
