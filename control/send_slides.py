# coding: utf-8
import PepperAPI
from PepperAPI import Info

PepperAPI.init("master")
# list_of_slides_text = [
# 	"Table of Contents\nOur Solar System\nHow stars are born\nThe Universe",
# 	"Our solar system contains planets and stars\nThe earth is the only planet that sustains life",
# 	"How are stars born"
# ]

# list_of_slides_text = [
# 'Introduction to Astronomy\n',
# 'Table of Contents\n' +
#     '« Overview of the solar system\n' +
#     '. Stars and galaxies\n' +
#     '« The Milky Way\n',
# 'Overview of the Solar System\n' +
#     '« The sun: size, structure, and energy production\n' +
#     '« The eight planets and their characteristics\n' +
#     '« Dwarf planets and other objects in the solar system\n' +
#     '« Exploration of the solar system by humans and robots\n',
# 'Stars and Galaxies\n'
#     '« Types of stars and their life cycles\n' +
#     '« Properties of galaxies\n' +
#     '« The Hubble classification system\n' +
#     '« The expansion of the universe\n'
# ]
list_of_slides_text = [
'Introduction to Astronomy\n',
'Overview of the Solar System\n' +
    '« The sun: size, structure, and energy production\n' +
    '« The eight planets and their characteristics\n' +
    '« Dwarf planets and other objects in the solar system\n' +
    '« Exploration of the solar system by humans and robots\n',
'Stars and Galaxies\n'
    '« Types of stars and their life cycles\n' +
    '« Properties of galaxies\n' +
    '« The Hubble classification system\n' +
    '« The expansion of the universe\n'
]

# Send text for all slides to Web one by one
number_of_slides = len(list_of_slides_text)
Info.Send("NumSlides", {"value": number_of_slides})	# tells NLP how many slides to receive
for slide in list_of_slides_text:
	Info.Send("Slides", {"text": slide})
