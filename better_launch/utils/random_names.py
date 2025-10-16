import random


# fmt: off
adjectives = [
    "airy", "arcane", "ashen", "auspicious", "benign", "bewildering", "bizarre", "bleary", "blithe", "briny", "brittle", "brusque", "bubbly", "bulbous", "buoyant", "celestial", "charred", "cheery", "chilly", "chipper", "clumsy", "cool", "crafty", "creaky", "crooked", "crumbly", "cryptic", "cryptid", "dainty", "dank", "dappled", "drizzly", "drowsy", "dusky", "dusty", "eclectic", "ecliptic", "effervescent", "eldritch", "electric", "energetic", "erratic", "esteemed", "ethereal", "excellent", "faintly", "fallow", "feathery", "feral", "flaking", "flickering", "fluffy", "fractious", "frenetic", "frothy", "fuzzy", "gelatinous", "gentle", "giddy", "gleaming", "gleeful", "glistening", "gooey", "goopy", "gossamer", "gritty", "gusty", "hilarious", "hoarse", "hungry", "inky", "iterative", "jangly", "jolly", "lanky", "lofty", "loopy", "lovely", "luminous", "lurid", "magnificent", "marooned", "meandering", "meek", "mellow", "melodious", "miraculous", "mirthful", "mossy", "mottled", "murine", "murky", "nifty", "nimble", "nocturnal", "oddball", "party", "peculiar", "peppy", "phantasmal", "phantastic", "pliant", "pliant", "plucky", "pondering", "primordial", "quaking", "questing", "raking", "ravenous", "relativistic", "reverent", "rumbling", "rustic", "sacred", "sanguine", "scintillant", "scrumptious", "searching", "serendipidous", "serpentine", "shaking", "shimmering", "silken", "silvery", "sinuous", "slippery", "slumberous", "smeary", "smudgy", "snarky", "snippy", "snoozy", "snug", "soggy", "somatic", "somber", "somnolent", "soothing", "sparkly", "speedy", "spindly", "splashing", "sprightly", "sprinkled", "spry", "squishy", "sublime", "swirly", "syrupy", "tangential", "tattered", "thready", "thundering", "touchy", "trembling", "tremulous", "turbulent", "twitchy", "unctuous", "unkempt", "unruly", "vehement", "verdant", "vexing", "vigilant", "volatile", "wandering", "weird", "whimsical", "wild", "wistful", "wobbly", "wondering", "wondrous", "zany", "zealou", ]
# fmt: on


# fmt: off
nouns = [
    "abyss", "acacia", "airship", "alchemist", "alpaca", "altar", "amulet", "anaconda", "ancestor", "andes", "anemone", "angel", "ant", "archipelago", "armadillo", "armor", "arrow", "ash", "ashram", "asteroid", "atom", "aurora", "autumn", "bamboo", "banner", "banyan", "baobab", "bard", "bark", "basilisk", "bat", "bauble", "bay", "bazaar", "beacon", "bead", "bear", "bee", "beetle", "bell", "bindi", "birch", "blade", "blood", "boar", "bog", "bone", "bow", "branch", "breath", "brook", "brush", "bud", "buddha", "bull", "butterfly", "calabash", "camel", "canoe", "canyon", "capybara", "carapace", "carp", "cassava", "caterpillar", "cathedral", "cattle", "cavern", "centaur", "chai", "chain", "chakra", "cherry", "chicken", "chime", "chimera", "cicada", "cinder", "circle", "circuit", "claw", "cloak", "cloth", "cloud", "cobra", "coconut", "coil", "comet", "condor", "coral", "core", "cow", "crab", "crane", "crater", "crest", "cricket", "crocodile", "crow", "crown", "crypt", "crystal", "current", "curry", "cuttlefish", "cyborg", "cypress", "dagger", "dawn", "deer", "deity", "delta", "demon", "depth", "desert", "djembe", "donkey", "doorway", "dragon", "dream", "driftwood", "drone", "drum", "dryad", "dune", "dusk", "dust", "easle", "echo", "eclipse", "eel", "elephant", "elk", "ember", "engine", "eon", "epoch", "expanse", "fabric", "falcon", "fan", "fang", "faun", "feather", "fen", "fern", "fiber", "firefly", "fish", "fjord", "flame", "flute", "fog", "footprint", "forest", "fox", "frog", "fungus", "galaxy", "gate", "gazelle", "gear", "gem", "ghost", "giant", "ginkgo", "giraffe", "glacier", "glyph", "goblin", "golem", "goose", "grain", "griffin", "griot", "grott", "grove", "guava", "guru", "halo", "hand", "harbor", "harp", "harpy", "heart", "helm", "henna", "herd", "heron", "hippo", "hollow", "homunculus", "hoof", "horizon", "horn", "hornet", "horse", "hut", "hydra", "hyena", "ibis", "idol", "imp", "incense", "iris", "island", "ivy", "jackal", "jade", "jaguar", "jellyfish", "juju" "jungle", "jungle", "kalimba", "kelp", "kimono", "knight", "koi", "kraken", "labyrinth", "lagoon", "lake", "lantern", "larva", "leaves", "lens", "leviathan", "lichen", "light", "lightning", "lily", "lion", "lizard", "llama", "lobster", "loom", "loop", "lotus", "lynx", "lyre", "macaw", "maelstrom", "mandala", "mango", "mantis", "mantle", "mark", "marsh", "mask", "meadow", "melody", "memory", "meteor", "midnight", "minotaur", "miracle", "mirage", "mire", "mirror", "mist", "molecule", "moment", "mongoose", "monk", "monolith", "monsoon", "moon", "moss", "moth", "mountain", "mouse", "mushroom", "nebula", "nerve", "newt", "noon", "nothingness", "nucleus", "nymph", "oak", "oasis", "obelisk", "obsidian", "ocean", "octopus", "okapi", "orb", "orchid", "otter", "owl", "ox", "pagoda", "palace", "palm" "panda", "panther", "party", "pattern", "peacock", "pebble", "penguin", "petal", "phantom", "phoenix", "pine", "pirate", "planet", "plateau", "pond", "portal", "prism", "pulse", "puma", "python", "quinoa", "rabbit", "radio", "rain", "rainforest", "rainmaker", "ram", "rangoli", "raven", "reed", "reef", "relic", "rhino", "rhythm", "rice", "ring", "ritual", "river", "robe", "robot", "rock", "rodent", "root", "rose", "ruin", "rune", "sahara", "saint", "sakura", "salamander", "sand", "sandstorm", "santa", "sap", "sari", "satellite", "savanna", "scale", "scarab", "scroll", "sea", "seal", "seed", "serpent", "shade", "shadow", "shaman", "shard", "shell", "shield", "ship", "shrimp", "shrine", "sigil", "signal", "silk", "sitar", "skull", "sky", "sloth", "slug", "smoke", "snail", "snake", "song", "spark", "sparrow", "spear", "specter", "sphere", "sphinx", "spice", "spiral", "spire", "spirit", "spore", "spring", "sprite", "squid", "star", "steppe", "stone", "storm", "stream", "string", "submarine", "summer", "sun", "sunstone", "swamp", "sword", "tabla", "tail", "tapestry", "tapir", "tea", "temple", "tentacle", "thistle", "thread", "threshold", "thunder", "tide", "tiger", "toad", "tomb", "tome", "torii", "totem", "totora", "toucan", "tower", "trace", "tree", "troll", "turban", "turtle", "twilight", "urchin", "valley", "veil", "vein", "vessel", "vicuna", "village", "villager", "vine", "vision", "voice", "volcano", "vulture", "wanderer", "wasp", "wave", "weave", "whale", "whirlpool", "whisper", "willow", "wind", "wing", "winter", "witch", "wizard", "wolf", "wonder", "worm", "wraith", "wyrm", "yak", "zebra", "zeppelin", "zither", ]
# fmt: on


class UniqueNameGenerator:
    def __init__(self, adjectives, nouns):
        self.adjectives = adjectives
        self.nouns = nouns
        # Pre-generate all possible combinations as indices
        self.available_indices = list(range(len(adjectives) * len(nouns)))
        random.shuffle(self.available_indices)
        self.next_index = 0

    def get_unique_name(self) -> str:
        if self.next_index >= len(self.available_indices):
            raise ValueError("No more unique names available")

        # Get the next index and convert to adj/noun pair
        idx = self.available_indices[self.next_index]
        self.next_index += 1

        adj_idx = idx // len(self.nouns)
        noun_idx = idx % len(self.nouns)

        return f"{self.adjectives[adj_idx]}{self.nouns[noun_idx]}"

    def reset(self):
        """Shuffle and reset the generator"""
        random.shuffle(self.available_indices)
        self.next_index = 0

    def remaining(self) -> int:
        """Return number of names still available"""
        return len(self.available_indices) - self.next_index


default_name_generator = UniqueNameGenerator(adjectives, nouns)


def get_unique_name() -> str:
    return default_name_generator.get_unique_name()


if __name__ == "__main__":
    print(f"{default_name_generator.remaining()} unique names!")

    discovered = set()
    for i in range(default_name_generator.remaining()):    
        n = default_name_generator.get_unique_name()
        if n in discovered:
            raise ValueError(f"Found duplicate: {n}")
        
        if i % 1000 == 0:
            print(n)
